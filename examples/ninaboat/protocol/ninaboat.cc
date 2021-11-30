#include "ninaboat.h"
#include <stdlib.h>
#include "mcu/watchdog.h"
#include <avr/interrupt.h>


#define HeaderEEM0 0xA0
#define HeaderEEM1 0x0A


namespace protocol {

typedef void (*do_reboot_t)(void);
const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1); // optiboot size

NinaBoat::NinaBoat():
    ser(mcu::Usart::get()),
    cout(ser),
    bq(devices::bq769x0::bq76940, 0x08, true)
{
    memset(&msg, 0, sizeof(msg));
}

void NinaBoat::setDebug(bool dbg) {
    m_Debug = dbg;
    bq.setDebug(m_Debug);
}

void NinaBoat::start() {
    loadSettings();
    applySettings();
    bq.update();
    bq.resetSOC(100);
    bq.enableCharging();
    bq.enableDischarging(); // todo
    print();
}


void NinaBoat::print() {
    if (!m_Debug) return;
    bq.printRegisters();
    debug_print();
}


void NinaBoat::alertISR() {
    bq.setAlertInterruptFlag();
    m_interruptFlag = true;
}

bool NinaBoat::update(mcu::Pin job) {
    uint32_t now = mcu::Timer::millis();
    
    if(m_interruptFlag || (uint32_t)(now - m_lastUpdate) >= 240) { // 500
        job = 1;
        if(m_interruptFlag) m_interruptFlag = false;
        uint8_t error = bq.update(); // should be called at least every 250 ms
        m_lastUpdate = now;

            // charging state
            if(bq.getBatteryCurrent() > (int16_t)m_Settings.idle_currentThres)
                packet.status |= (1 << 6); // charging
                else if(bq.getBatteryCurrent() < (int16_t)m_Settings.idle_currentThres / 2)
                    packet.status &= ~(1 << 6);
                
                if(error & STAT_OV) {
                    packet.status |= (1 << 9); // overvoltage
                    error &= ~STAT_OV;
                }
                else
                    packet.status &= ~(1 << 9);
                
                uint16_t batVoltage = bq.getBatteryVoltage() / 10;
                if(batVoltage > packet.max_voltage)
                    packet.max_voltage = batVoltage;
                
                int16_t batCurrent = bq.getBatteryCurrent() / 10;
                if(batCurrent > 0 && (uint16_t)batCurrent > packet.max_charge_current)
                    packet.max_charge_current = batCurrent;
                else if(batCurrent < 0 && (uint16_t)-batCurrent > packet.max_discharge_current)
                    packet.max_discharge_current = -batCurrent;
                
                packet.capacity_left = packet.design_capacity * bq.getSOC() / 100.0;
                packet.percent_left = bq.getSOC();
                packet.current = -batCurrent;
                packet.voltage = batVoltage;
                packet.temperature[0] = bq.getTemperatureDegC(1) + 20.0;
                packet.temperature[1] = bq.getTemperatureDegC(2) + 20.0;
                
                if(bq.getHighestTemperature() > (m_Settings.temp_maxDischargeC - 3) * 10)
                    packet.status |= (1 << 10); // overheat
                    else
                        packet.status &= ~(1 << 10);
                    
                    if(bq.batCycles_) {
                        packet.num_cycles += bq.batCycles_;
                        bq.batCycles_ = 0;
                        m_Settings.num_cycles = packet.num_cycles;
                        saveSettings();

                    }
                    
                    if(bq.chargedTimes_) {
                        packet.num_charged += bq.chargedTimes_;
                        m_Settings.num_charged = packet.num_charged;
                        bq.chargedTimes_ = 0;
                    }
                    
                    uint8_t numCells = bq.getNumberOfConnectedCells();
                    for(uint8_t i = 0; i < numCells; i++)
                        packet.cell_voltages[i] = bq.getCellVoltage(i);
                    
                    // cell voltage difference too big
                    uint16_t bigDelta = bq.getMaxCellVoltage() - bq.getMinCellVoltage();
                    if(bigDelta > 100)
                        error = 1;
                    
                    if(error)
                        packet.status &= ~1;
                    else
                        packet.status |= 1;

        
        if(m_oldMillis > now)
            m_millisOverflows++;
        m_oldMillis = now;
        job = 0;
    }
    
    
    return Recv();
}




void NinaBoat::onMessage(NinaBoatMessage &msg) {
    // Enable TX
    UCSR0B |= (1 << TXEN0);
    
    if(msg.addr != M365BMS_RADDR) return;
    
    if(msg.mode == 0x01 || msg.mode == 0xF1) {
        
        if(msg.length != 3) return;
        uint16_t ofs = (uint16_t)msg.offset * 2; // word aligned
        uint8_t sz = msg.data[0];
        if(sz > sizeof(NinaBoatMessage::data)) return;
        msg.addr = M365BMS_WADDR;
        msg.length = 2 + sz;
        
        if(msg.mode == 0x01) {
            if((ofs + sz) > sizeof(packet)) return;
            memcpy(&msg.data, &((uint8_t *)&packet)[ofs], sz);
        } else if(msg.mode == 0xF1) {
            if((ofs + sz) > sizeof(m_Settings)) return;
            memcpy(&msg.data, &((uint8_t *)&m_Settings)[ofs], sz);
        }
        Send(msg);
        
    } else if(msg.mode == 0x03 || msg.mode == 0xF3) {
        uint16_t ofs = (uint16_t)msg.offset * 2; // word aligned
        uint8_t sz = msg.length - 2;
        
        if(msg.mode == 0x03) {
            if((ofs + sz) > sizeof(packet)) return;
            memcpy(&((uint8_t *)&packet)[ofs], &msg.data, sz);
        } else if(msg.mode == 0xF3) {
            if((ofs + sz) > sizeof(m_Settings)) return;
            memcpy(&((uint8_t *)&m_Settings)[ofs], &msg.data, sz);
        }
        
    } else if(msg.mode == 0xFA) {
        switch(msg.offset) {
            case 1: applySettings(); break;
            case 2: loadSettings(); break; // EEPROM.get(0, m_Settings); break;
            case 3: saveSettings(); break; //EEPROM.put(0, m_Settings); break;
#if BQ769X0_DEBUG
            case 4: m_Debug = msg.data[0]; break;
            case 5: debug_print(); break;
#endif
            case 6: bq.disableDischarging(); bq.disableCharging(); break;
            case 7: bq.enableDischarging(); bq.enableCharging(); break;
            //case 8: digitalWrite(BMS_VDD_EN_PIN, LOW); break;
            //case 9: digitalWrite(BMS_VDD_EN_PIN, HIGH); break;
#if BQ769X0_DEBUG
            case 10: {
                // test watchdog
                for (;;) { (void)0; }
            } break;
#endif
            case 11: {
                // restart to bootloader
                mcu::Watchdog::disable();
                cli();
                TCCR0A = 0;
                TCCR1A = 0;
                TCCR2A = 0; // make sure interrupts are off and timers are reset.
                MCUSR = 0;
                do_reboot();
            }
        }
    }
}

void NinaBoat::Send(NinaBoatMessage &msg) {
    
    msg.checksum = (uint16_t)msg.length + msg.addr + msg.mode + msg.offset;
    
    
    ser.write(msg.header[0]);
    ser.write(msg.header[1]);
    ser.write(msg.length);
    ser.write(msg.addr);
    ser.write(msg.mode);
    ser.write(msg.offset);
    for(uint8_t i = 0; i < msg.length - 2; i++)
    {
        ser.write(msg.data[i]);
        msg.checksum += msg.data[i];
    }
    
    msg.checksum ^= 0xFFFF;
    ser.write(msg.checksum & 0xFF);
    ser.write((msg.checksum >> 8) & 0xFF);
}

bool NinaBoat::Recv() {
    
    bool result = false;
    
    while(ser.avail()) {
        result = true;
        
        // 100ms timeout
        if(mcu::Timer::millis() >= begin + 100) recvd = 0;
        
        uint8_t byte = ser.read();
        recvd++;
        
        switch(recvd) {
            
            case 1: {
                if(byte != 0x55) { // header1 mismatch
                    recvd = 0;
                    break;
                }
                msg.header[0] = byte;
                begin = mcu::Timer::millis();
            } break;
            
            case 2:
            {
                if(byte != 0xAA) { // header2 mismatch
                    recvd = 0;
                    break;
                }
                msg.header[1] = byte;
            } break;
            
            case 3: // length
            {
                if(byte < 2)
                { // too small
                    recvd = 0;
                    break;
                }
                
                msg.length = byte;
                checksum = byte;
            } break;
            
            case 4: // addr
            {
                if(byte != M365BMS_RADDR)
                { // we're not the receiver of this message
                    recvd = 0;
                    break;
                }
                
                msg.addr = byte;
                checksum += byte;
            } break;
            
            case 5: // mode
            {
                msg.mode = byte;
                checksum += byte;
            } break;
            
            case 6: // offset
            {
                msg.offset = byte;
                checksum += byte;
            } break;
            
            default:
            {
                if(recvd - 7 < msg.length - 2)
                { // data
                    msg.data[recvd - 7] = byte;
                    checksum += byte;
                }
                else if(recvd - 7 - msg.length + 2 == 0)
                { // checksum LSB
                    msg.checksum = byte;
                }
                else
                { // checksum MSB and transmission finished
                    msg.checksum |= (uint16_t)byte << 8;
                    checksum ^= 0xFFFF;
                    
                    if(checksum != msg.checksum)
                    { // invalid checksum
                        recvd = 0;
                        break;
                    }
                    
                    onMessage(msg);
                    recvd = 0;
                }
            } break;
        }
    }
    return result;
}







#if BQ769X0_DEBUG
void NinaBoat::debug_print() {

    
    uint32_t uptime = m_millisOverflows * (0xffffffffLL / 1000UL);
    uptime += mcu::Timer::millis() / 1000;
    cout << stream::Flags::PGM << PSTR("uptime: ") << uptime << EOL;
    cout << stream::Flags::PGM << 
    PSTR("Battery voltage: ") << bq.getBatteryVoltage() << stream::Flags::PGM <<
    PSTR(" (") << bq.getBatteryVoltage(true) << stream::Flags::PGM << PSTR(")\r\n");
    
    cout << stream::Flags::PGM <<
    PSTR("Battery current: ") << bq.getBatteryCurrent() << stream::Flags::PGM <<
    PSTR(" (") << bq.getBatteryCurrent(true) << stream::Flags::PGM << PSTR(")\r\n"); // TODO
    
    cout << stream::Flags::PGM << PSTR("SOC: ") << bq.getSOC() << EOL; 
    
    cout << stream::Flags::PGM << PSTR("Temperature: ") <<
    bq.getTemperatureDegC(0) << ' ' <<
    bq.getTemperatureDegC(1) << ' ' <<
    bq.getTemperatureDegC(2) << EOL;
    uint8_t numCells = bq.getNumberOfCells();
    cout << stream::Flags::PGM << PSTR("Balancing status: ") << bq.getBalancingStatus() << EOL;
    cout << stream::Flags::PGM << PSTR("Cell voltages (") <<
    bq.getNumberOfConnectedCells() << stream::Flags::PGM <<
    PSTR(" / ") << numCells << stream::Flags::PGM << PSTR("):") << EOL;
    
    for(uint8_t i = 0; i < numCells; i++) {
        cout <<  bq.getCellVoltage_(i) << stream::Flags::PGM << PSTR(" (") <<
        bq.getCellVoltage_(i, true) << stream::Flags::PGM << PSTR(")");
        if(i != numCells - 1) cout << stream::Flags::PGM << PSTR(", ");
    }
    
    cout << stream::Flags::PGM << PSTR("\r\n\r\nCell V: Min: ") << bq.getMinCellVoltage();
    cout << stream::Flags::PGM << PSTR(" | Avg: ") << bq.getAvgCellVoltage();
    cout << stream::Flags::PGM << PSTR(" | Max: ") << bq.getMaxCellVoltage();
    cout << stream::Flags::PGM << PSTR(" | Delta: ") << bq.getMaxCellVoltage() - bq.getMinCellVoltage();
    cout << stream::Flags::PGM << PSTR("\r\n\r\n         maxVoltage: ") << packet.max_voltage;
    cout << stream::Flags::PGM << PSTR("\r\nmaxDischargeCurrent: ") << packet.max_discharge_current;
    cout << stream::Flags::PGM << PSTR("\r\n   maxChargeCurrent: ") << packet.max_charge_current;
    cout << stream::Flags::PGM << PSTR("\r\n\r\nXREADY errors: ") << bq.errorCounter_[bq.ERROR_XREADY];
    cout << stream::Flags::PGM << PSTR("\r\n ALERT errors: ") << bq.errorCounter_[bq.ERROR_ALERT];
    cout << stream::Flags::PGM << PSTR("\r\n   UVP errors: ") << bq.errorCounter_[bq.ERROR_UVP];
    cout << stream::Flags::PGM << PSTR("\r\n   OVP errors: ") << bq.errorCounter_[bq.ERROR_OVP];
    cout << stream::Flags::PGM << PSTR("\r\n   SCD errors: ") << bq.errorCounter_[bq.ERROR_SCD];
    cout << stream::Flags::PGM << PSTR("\r\n   OCD errors: ") << bq.errorCounter_[bq.ERROR_OCD];
    cout << stream::Flags::PGM << PSTR("\r\n\r\nDISCHG TEMP errors: ") << bq.errorCounter_[bq.ERROR_USER_DISCHG_TEMP];
    cout << stream::Flags::PGM << PSTR("\r\n   CHG TEMP errors: ") << bq.errorCounter_[bq.ERROR_USER_CHG_TEMP];
    cout << stream::Flags::PGM << PSTR("\r\n    CHG OCD errors: ") << bq.errorCounter_[bq.ERROR_USER_CHG_OCD] << EOL;
}
#endif





void NinaBoat::applySettings() {
    bq.setBatteryCapacity(
        m_Settings.capacity,
        m_Settings.nominal_voltage,
        m_Settings.full_voltage
    );
    
    bq.setShuntResistorValue(m_Settings.shuntResistor_uOhm);
    bq.setThermistorBetaValue(m_Settings.thermistor_BetaK);
    bq.setTemperatureLimits(
        m_Settings.temp_minDischargeC,
        m_Settings.temp_maxDischargeC,
        m_Settings.temp_minChargeC,
        m_Settings.temp_maxChargeC
    );
    bq.setShortCircuitProtection(m_Settings.SCD_current, m_Settings.SCD_delay);
    bq.setOvercurrentChargeProtection(m_Settings.OCD_current, m_Settings.OCD_delay);
    bq.setOvercurrentDischargeProtection(m_Settings.ODP_current, m_Settings.ODP_delay);
    bq.setCellUndervoltageProtection(m_Settings.UVP_voltage, m_Settings.UVP_delay);
    bq.setCellOvervoltageProtection(m_Settings.OVP_voltage, m_Settings.OVP_delay);    
    bq.setBalancingThresholds(
        m_Settings.balance_minIdleTime,
        m_Settings.balance_minVoltage,
        m_Settings.balance_maxVoltageDiff
    );
    bq.setIdleCurrentThreshold(m_Settings.idle_currentThres);
     
    bq.setAutoBalancing((bool)m_Settings.balance_enabled);
    
    bq.setBalanceCharging(true);
    
    bq.adjADCPackOffset(m_Settings.adcPackOffset);
    bq.adjADCCellsOffset(m_Settings.adcCellsOffset);
    
    strncpy(packet.serial, m_Settings.serial, sizeof(packet.serial));
    packet.design_capacity = m_Settings.capacity;
    packet.real_capacity = m_Settings.capacity;
    packet.nominal_voltage = m_Settings.nominal_voltage;
    packet.date = m_Settings.date;
    packet.num_cycles = m_Settings.num_cycles;
    packet.num_charged = m_Settings.num_charged;
}

BMSSettings EEMEM bmset_eep;

void NinaBoat::loadSettings() {
//     eeprom_read_block(&m_Settings, &bmset_eep, sizeof(m_Settings));
//     if ((HeaderEEM0 != m_Settings.header[0]) || (HeaderEEM1 != m_Settings.header[1])) {
        loadSettingsDefault();
//         saveSettings();
//     }
}

void NinaBoat::saveSettings() {
//     eeprom_write_block(&m_Settings, &bmset_eep, sizeof(m_Settings));
}

void NinaBoat::loadSettingsDefault() {
    m_Settings.header[0] = HeaderEEM0;
    m_Settings.header[1] = HeaderEEM1;
    m_Settings.version = 1;
    m_Settings.serial[0] = 'L';
    m_Settings.serial[1] = 'a'; 
    m_Settings.serial[2] = 'n';
    m_Settings.serial[3] = 't';
    m_Settings.serial[4] = 'h';
    m_Settings.serial[5] = 'i';
    m_Settings.serial[6] = ' ';
    m_Settings.serial[7] = '1';
    m_Settings.serial[8] = '2';
    m_Settings.serial[9] = 's';
    m_Settings.serial[10] = '5';
    m_Settings.serial[11] = 'p';
    m_Settings.serial[12] = '\0';
    m_Settings.capacity = 13800; // mAh Put here your real milliamps if you have an amp tester look at your real battery at the output
    m_Settings.nominal_voltage = 3600; // mV
    m_Settings.full_voltage = 4175; // mV
    m_Settings.num_cycles = 0;
    m_Settings.num_charged = 0;
    m_Settings.date = (20 << 9) | (11 << 5) | 10; // MSB (7 bits year, 4 bits month, 5 bits day) LSB
    // setShuntResistorValue
    m_Settings.shuntResistor_uOhm = 1000;
    // setThermistorBetaValue
    m_Settings.thermistor_BetaK = 3435;
    // setTemperatureLimits
    m_Settings.temp_minDischargeC = -20; // °C
    m_Settings.temp_maxDischargeC = 60; // °C
    m_Settings.temp_minChargeC = 0; // °C
    m_Settings.temp_maxChargeC = 45; // °C
    // setShortCircuitProtection
    m_Settings.SCD_current = 80000; // mA
    m_Settings.SCD_delay = 200; // us
    // setOvercurrentChargeProtection
    m_Settings.OCD_current = 5000; // mA
    m_Settings.OCD_delay = 3000; // ms
    // setOvercurrentDischargeProtection
    m_Settings.ODP_current = 35000; // mA
    m_Settings.ODP_delay = 1280; // ms
    // setCellUndervoltageProtection
    m_Settings.UVP_voltage = 2800; // mV
    m_Settings.UVP_delay = 2; // s
    // setCellOvervoltageProtection
    m_Settings.OVP_voltage = 4200; // mV
    m_Settings.OVP_delay = 2; // s
    // setBalancingThresholds
    m_Settings.balance_minIdleTime = 1800; // s
    m_Settings.balance_minVoltage = 3600; // mV
    m_Settings.balance_maxVoltageDiff = 10; // mV
    // setIdleCurrentThreshold
    m_Settings.idle_currentThres = 500; // mA
    // enableAutoBalancing
    m_Settings.balance_enabled = 1;
    // adjADCPackOffset
    m_Settings.adcPackOffset = 0;
    // adjADCCellsOffset
    memset(m_Settings.adcCellsOffset, 0, sizeof(m_Settings.adcCellsOffset));
    
}



}
