#include "console.h"
#include <stdlib.h>
#include "mcu/watchdog.h"
#include <avr/interrupt.h>



namespace {
extern "C" uint16_t get_free_mem() {
    extern int16_t __heap_start, *__brkval;
    int16_t v;
    int16_t Free__Ram = (int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval);
    return (uint16_t)abs(Free__Ram);
}
}

#define BackSpace  0x08
#define Delete 0x7F

namespace protocol {

void Console::write_help(stream::OutputStream &out, const char *cmd, const char *help) {
    out << "  " << stream::PGM << cmd;
    uint8_t len = strlen_P(cmd);
    while (len++ < 20) out << ' ';
    out << stream::PGM << help << EOL;
}
    

Console::Console():
    ser(mcu::Usart::get()),
    cout(ser),
    bq(devices::bq769x0::bq76940, 0x08, true)
{
    // memset(&a, 0, sizeof(a));
}

// void Console::setDebug(bool dbg) {
//     m_Debug = dbg;
//     bq.setDebug(m_Debug);
// }

void Console::start() {
    loadSettings();
    applySettings();
    bq.update();
    bq.resetSOC(100);
    bq.enableCharging();
    bq.enableDischarging(); // todo
    bq.printRegisters();
    bq.setDebug(false);
    debug_print();
}


// void Console::print() {
//     if (!m_Debug) return;
//     bq.printRegisters();
//     debug_print();
// }


void Console::alertISR() {
    bq.setAlertInterruptFlag();
    m_interruptFlag = true;
}

bool Console::update(mcu::Pin job) {
    uint32_t now = mcu::Timer::millis();
    
    if(m_interruptFlag || (uint32_t)(now - m_lastUpdate) >= 240) { // 500
        job = 1;
        if(m_interruptFlag) m_interruptFlag = false;
        uint8_t error = bq.update(); // should be called at least every 250 ms
        m_lastUpdate = now;
        /*
            // charging state
            if(bq.getBatteryCurrent()       > (int16_t)m_Settings.idle_currentThres) packet.status |= (1 << 6); // charging
            else if(bq.getBatteryCurrent()  < (int16_t)m_Settings.idle_currentThres / 2) packet.status &= ~(1 << 6);
            if(error & STAT_OV) { packet.status |= (1 << 9); error &= ~STAT_OV; } // overvoltage
            else packet.status &= ~(1 << 9);
            uint16_t batVoltage = bq.getBatteryVoltage() / 10;
            if(batVoltage > packet.max_voltage) packet.max_voltage = batVoltage;
            int16_t batCurrent = bq.getBatteryCurrent() / 10;
            if(batCurrent > 0 && (uint16_t)batCurrent > packet.max_charge_current)
                packet.max_charge_current = batCurrent;
            else if(batCurrent < 0 && (uint16_t)-batCurrent > packet.max_discharge_current)
                packet.max_discharge_current = -batCurrent;
                0
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

        */
        if(m_oldMillis > now)
            m_millisOverflows++;
        m_oldMillis = now;
        job = 0;
    }
    

    return Recv();
}

void Console::command_apply() { applySettings(); }
void Console::command_restore() { loadSettings(); }
void Console::command_save() { saveSettings(); }
void Console::command_print() { debug_print(); }
void Console::command_bqregs() { bq.printRegisters(); }
void Console::command_bqdbg() { if (param_len) { bq.setDebug((bool)atoi(param)); } }
void Console::command_wdtest() { for (;;) { (void)0; } }
void Console::command_freemem() { cout << stream::Flags::PGM << PSTR(" Free RAM:") << get_free_mem() << EOL; }
typedef void (*do_reboot_t)(void);
const do_reboot_t do_reboot = (do_reboot_t)((FLASHEND - 511) >> 1); // optiboot size

void Console::command_bootloader() {
    // restart to bootloader
    mcu::Watchdog::disable();
    cli();
    TCCR0A = 0;
    TCCR1A = 0;
    TCCR2A = 0; // make sure interrupts are off and timers are reset.
    MCUSR = 0;
    do_reboot();
}

void Console::command_format_EEMEM() {
    for (int i = 0 ; i < E2END + 1 ; i++) {
        eeprom_write_byte((uint8_t*)i, 0xff);
        ser.write('.');
        mcu::Watchdog::reset();
    }
    cout << EOL;
}

char const STR_CMD_APPLY[] PROGMEM   = "apply";
char const STR_CMD_APPLY_HLP[] PROGMEM   = "apply settings";

char const STR_CMD_RESTORE[] PROGMEM   = "restore";
char const STR_CMD_RESTORE_HLP[] PROGMEM   = "restore defaults settings";

char const STR_CMD_SAVE[] PROGMEM   = "save";
char const STR_CMD_SAVE_HLP[] PROGMEM   = "save current settings";

char const STR_CMD_PRINT[] PROGMEM   = "print";
char const STR_CMD_PRINT_HLP[] PROGMEM   = "print status";

char const STR_CMD_WDTEST[] PROGMEM   = "wdtest";
char const STR_CMD_WDTEST_HLP[] PROGMEM   = "test watchdog";

char const STR_CMD_BOOTLOADER[] PROGMEM   = "bootloader";
char const STR_CMD_BOOTLOADER_HLP[] PROGMEM   = "jump to bootloader";

char const STR_CMD_FREEMEM[] PROGMEM   = "freemem";
char const STR_CMD_FREEMEM_HLP[] PROGMEM   = "show free memory in heap";

char const STR_CMD_EPFORMAT[] PROGMEM   = "format";
char const STR_CMD_EPFORMAT_HLP[] PROGMEM   = "formating EEPROM (load defaults settings on next boot)";

char const STR_CMD_HELP[] PROGMEM   = "help";
char const STR_CMD_HELP_HLP[] PROGMEM   = "this 'help'";


// // // // // // char const STR_CMD_HELP[] PROGMEM   = "help";
// // // // // // char const STR_CMD_HELP_HLP[] PROGMEM   = "this 'help'";


char const STR_CMD_BQDBG[] PROGMEM   = "bqdbg";
char const STR_CMD_BQDBG_HLP[] PROGMEM   = "enable (1) or disable (0) debug events on BQ769x0";

char const STR_CMD_BQREGS[] PROGMEM   = "bqregs";
char const STR_CMD_BQREGS_HLP[] PROGMEM   = "print regs in BQ769x0";



void Console::commandHelp() {
    cout << stream::PGM << PSTR("Available commands:\r\n") << EOL;
    write_help(cout, STR_CMD_APPLY,     STR_CMD_APPLY_HLP);
    write_help(cout, STR_CMD_RESTORE,   STR_CMD_RESTORE_HLP);
    write_help(cout, STR_CMD_SAVE,      STR_CMD_SAVE_HLP);
    write_help(cout, STR_CMD_BQDBG,     STR_CMD_BQDBG_HLP);
    write_help(cout, STR_CMD_BQREGS,    STR_CMD_BQREGS_HLP);
    write_help(cout, STR_CMD_PRINT,     STR_CMD_PRINT_HLP);
    write_help(cout, STR_CMD_WDTEST,    STR_CMD_WDTEST_HLP);
    write_help(cout, STR_CMD_BOOTLOADER,STR_CMD_BOOTLOADER_HLP);
    write_help(cout, STR_CMD_FREEMEM,   STR_CMD_FREEMEM_HLP);
    write_help(cout, STR_CMD_EPFORMAT,  STR_CMD_EPFORMAT_HLP);
    write_help(cout, STR_CMD_HELP,      STR_CMD_HELP_HLP);
    cout << EOL;
}


void Console::compare_cmd(const char *name_P, SerialCommandHandler handler) {
    const uint8_t cmd_len = strlen_P(name_P);
    if (strncmp_P(handle_buffer, name_P, cmd_len) == 0) {
        if ((handle_len > cmd_len) && (handle_buffer[cmd_len] == ' ')) {
            param = handle_buffer + cmd_len + 1;
            param_len = handle_len - cmd_len - 1;
            (this->*handler)(); // cmd with arg
        } else {
            param = nullptr;
            param_len = 0;
            (this->*handler)(); // single cmd
        }
        handle_result = true;
    }
}

bool Console::handleCommand(const char *buffer, const uint8_t len) {
    if (buffer[0] == 0) return false;
    handle_result = false;
    handle_buffer = buffer;
    handle_len = len;    
    compare_cmd(STR_CMD_APPLY,      &Console::command_apply);
    compare_cmd(STR_CMD_RESTORE,    &Console::command_restore);
    compare_cmd(STR_CMD_SAVE,       &Console::command_save);
    compare_cmd(STR_CMD_BQDBG,      &Console::command_bqdbg);
    compare_cmd(STR_CMD_BQREGS,     &Console::command_bqregs);
    compare_cmd(STR_CMD_PRINT,      &Console::command_print);
    compare_cmd(STR_CMD_WDTEST,     &Console::command_wdtest);
    compare_cmd(STR_CMD_BOOTLOADER, &Console::command_bootloader);
    compare_cmd(STR_CMD_FREEMEM,    &Console::command_freemem);
    compare_cmd(STR_CMD_EPFORMAT,   &Console::command_format_EEMEM);
    compare_cmd(STR_CMD_HELP,       &Console::commandHelp);
    if (!handle_result) { cout << stream::PGM << PSTR("Unknown command. Try 'help'") << EOL; }
    return handle_result;
}

bool Console::Recv() {
    bool result = false;
    char ch;
    if (state == CONSOLE_STARTUP) {
        state = CONSOLE_ACCUMULATING;
    } else if (state == CONSOLE_ACCUMULATING) {
        while (ser.avail()) {
            
// // // // //             ser.enableTX();
            result = true;
            ch = ser.read();
            if (len == 0 && ch == '$') echo = false;
            if (echo) {
                ser.write(ch);
                if (ch == CR) ser.write(LF);
            }
            if (ch == BackSpace || ch == Delete) {
                if (len) buffer[--len] = 0;
            } else {
                if (ch != LF) buffer[len++] = ch;
            }
            if (len == CONSOLE_BUFFER || ch == CR) {
                buffer[--len] = 0;
                state = CONSOLE_COMMAND;
                break;
            }
        }
    } else if (state == CONSOLE_COMMAND) {
        handleCommand(buffer, len);
        cout << "BMS>";
        len = 0;
        echo = true;
        state = CONSOLE_ACCUMULATING;
    }
    return result;
}


void Console::debug_print() {

    
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
//     cout << stream::Flags::PGM << PSTR("\r\n\r\n         maxVoltage: ") << packet.max_voltage;
//     cout << stream::Flags::PGM << PSTR("\r\nmaxDischargeCurrent: ") << packet.max_discharge_current;
//     cout << stream::Flags::PGM << PSTR("\r\n   maxChargeCurrent: ") << packet.max_charge_current;
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


void Console::applySettings() {
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
    
//     strncpy(packet.serial, m_Settings.serial, sizeof(packet.serial));
//     packet.design_capacity = m_Settings.capacity;
//     packet.real_capacity = m_Settings.capacity;
//     packet.nominal_voltage = m_Settings.nominal_voltage;
//     packet.date = m_Settings.date;
//     packet.num_cycles = m_Settings.num_cycles;
//     packet.num_charged = m_Settings.num_charged;
}

BMSSettings EEMEM bmset_eep;

void Console::loadSettings() {
//     eeprom_read_block(&m_Settings, &bmset_eep, sizeof(m_Settings));
//     if ((HeaderEEM0 != m_Settings.header[0]) || (HeaderEEM1 != m_Settings.header[1])) {
        loadSettingsDefault();
//         saveSettings();
//     }
}

void Console::saveSettings() {
//     eeprom_write_block(&m_Settings, &bmset_eep, sizeof(m_Settings));
}

void Console::loadSettingsDefault() {
//     m_Settings.header[0] = HeaderEEM0;
//     m_Settings.header[1] = HeaderEEM1;
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
