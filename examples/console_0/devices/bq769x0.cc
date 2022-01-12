/* Battery management system based on bq769x0 for ARM mbed
 * Copyright (c) 2015-2018 Martin Jäger (www.libre.solar)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h>     // log for thermistor calculation
#include "bq769x0.h"

#include <string.h>
#include "mcu/timer.h"
#include <util/delay.h>
#include <stdint.h>
#include <stdlib.h>
#include <limits.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <stdint.h>

namespace devices {

const char *byte2char(int x) {
    static char b[9];
    b[0] = '\0';
    int z;
    for (z = 128; z > 0; z >>= 1) {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return b;
}

uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData) {
    uint8_t data = inCrc ^ inData;
    for (uint8_t i = 0; i < 8; i++) {
        if ((data & 0x80) != 0) {
            data <<= 1;
            data ^= 0x07;
        } else {
            data <<= 1;
        }
    }
    return data;
}

bq769x0::bq769x0(bq769_conf &_conf, bq769_data &_data):
    cout(mcu::Usart::get()),
    conf(_conf),
    data(_data)
{
    memset(&_data, 0, sizeof(_data));
    data.alertInterruptFlag_ = true;
}

void bq769x0::begin() {
    // test communication
    while (true) {
        // should be set to 0x19 according to datasheet
        writeRegister(CC_CFG, 0x19);
        if (readRegister(CC_CFG) == 0x19) break;
        cout << stream::Flags::PGM << PSTR("bq769x0 CFG error!\r\n");
        _delay_ms(125);
    }
    // initial settings for bq769x0
    writeRegister(SYS_CTRL1, 0b00011000);  // switch external thermistor and ADC on
    writeRegister(SYS_CTRL2, 0b01000000);  // switch CC_EN on
    // reset balance registers just in case
    writeRegister(CELLBAL1, 0x0);
    writeRegister(CELLBAL2, 0x0);
    writeRegister(CELLBAL3, 0x0);
    // get ADC offset and gain
    data.adcOffset_ = readRegister(ADCOFFSET);
    data.adcGain_ = 365 + (
        ((readRegister(ADCGAIN1) & 0b00001100) << 1) |
        ((readRegister(ADCGAIN2) & 0b11100000) >> 5)
    ); // uV/LSB
}


//-----------------------------------------------------------------------------

// void bq769x0::setDebug(bool debug) { conf.m_Debug = debug; }

//-----------------------------------------------------------------------------

//----------------------------------------------------------------------------
// Boot IC by pulling the boot pin TS1 high for some ms

// void bq769x0::boot(uint8_t bootPin)
// {
//     pinMode(bootPin, OUTPUT);
//     digitalWrite(bootPin, HIGH);
//     delay(5); // wait 5 ms for device to receive boot signal (datasheet: max. 2 ms)
//     pinMode(bootPin, INPUT); // don't disturb temperature measurement
//     delay(10); // wait for device to boot up completely (datasheet: max. 10 ms)
// }

//----------------------------------------------------------------------------
// Fast function to check whether BMS has an error
// (returns 0 if everything is OK)

uint8_t bq769x0::checkStatus() {

    if (data.alertInterruptFlag_ || errorStatus_.regByte) {
        regSYS_STAT_t sys_stat;
        sys_stat.regByte = readRegister(SYS_STAT);

        // first check, if only a new CC reading is available
        if (sys_stat.bits.CC_READY == 1) {
            if(conf.m_Debug) cout << '.'; // cout << stream::Flags::PGM << PSTR("Interrupt: CC ready\r\n");
            updateCurrent();  // automatically clears CC ready flag
        }

        // Serious error occured
        if (sys_stat.regByte & STAT_FLAGS) {
            if (!errorStatus_.bits.DEVICE_XREADY && sys_stat.bits.DEVICE_XREADY) { // XR error
                data.chargingEnabled_ = data.dischargingEnabled_ = false;
                data.chargingDisabled_ |= (1 << ERROR_XREADY);
                data.dischargingDisabled_ |= (1 << ERROR_XREADY);
                errorCounter_[ERROR_XREADY]++;
                data.errorTimestamps_[ERROR_XREADY] = mcu::Timer::millis();
                if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: XREADY\r\n");
            }
            if (!errorStatus_.bits.OVRD_ALERT && sys_stat.bits.OVRD_ALERT) { // Alert error
                data.chargingEnabled_ = data.dischargingEnabled_ = false;
                data.chargingDisabled_ |= (1 << ERROR_ALERT);
                data.dischargingDisabled_ |= (1 << ERROR_ALERT);

                errorCounter_[ERROR_ALERT]++;
                data.errorTimestamps_[ERROR_ALERT] = mcu::Timer::millis();

                if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: ALERT\r\n");

            }
            if (sys_stat.bits.UV) { // UV error
                data.dischargingEnabled_ = false;
                data.dischargingDisabled_ |= (1 << ERROR_UVP);

                errorCounter_[ERROR_UVP]++;
                data.errorTimestamps_[ERROR_UVP] = mcu::Timer::millis();

                if(conf.m_Debug)
                    cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: UVP\r\n");

            }
            if (sys_stat.bits.OV) { // OV error
                data.chargingEnabled_ = false;
                data.chargingDisabled_ |= (1 << ERROR_OVP);

                errorCounter_[ERROR_OVP]++;
                data.errorTimestamps_[ERROR_OVP] = mcu::Timer::millis();

                if(conf.m_Debug)
                    cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: OVP\r\n");

            }
            if (sys_stat.bits.SCD) { // SCD
                data.dischargingEnabled_ = false;
                data.dischargingDisabled_ |= (1 << ERROR_SCD);

                errorCounter_[ERROR_SCD]++;
                data.errorTimestamps_[ERROR_SCD] = mcu::Timer::millis();

                if(conf.m_Debug)
                    cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: SCD\r\n");


            }
            if (sys_stat.bits.OCD) { // OCD
                data.dischargingEnabled_ = false;
                data.dischargingDisabled_ |= (1 << ERROR_OCD);

                errorCounter_[ERROR_OCD]++;
                data.errorTimestamps_[ERROR_OCD] = mcu::Timer::millis();

                if(conf.m_Debug)
                    cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: OCD\r\n");


            }

            errorStatus_.regByte = sys_stat.regByte;
        }
        else {
            errorStatus_.regByte = 0;
        }
    }
    return errorStatus_.regByte;
}

//----------------------------------------------------------------------------
// tries to clear errors which have been found by checkStatus()

void bq769x0::clearErrors() {
    
    if(errorStatus_.bits.DEVICE_XREADY) {
        // datasheet recommendation: try to clear after waiting a few seconds
        if((uint32_t)(mcu::Timer::millis() - data.errorTimestamps_[ERROR_XREADY]) > 3UL * 1000UL) {
            if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear XREADY error\r\n");
            writeRegister(SYS_STAT, STAT_DEVICE_XREADY);
            enableCharging(1 << ERROR_XREADY);
            enableDischarging(1 << ERROR_XREADY);
            errorStatus_.bits.DEVICE_XREADY = 0;
        }
    }
    
    if(errorStatus_.bits.OVRD_ALERT) {
        if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear ALERT error\r\n");
        writeRegister(SYS_STAT, STAT_OVRD_ALERT);
        enableCharging(1 << ERROR_ALERT);
        enableDischarging(1 << ERROR_ALERT);
        errorStatus_.bits.OVRD_ALERT = 0;
    }
    
    if(errorStatus_.bits.UV) {
        if(data.cellVoltages_[idCellMinVoltage_] > minCellVoltage_) {
            if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear UVP error\r\n");
            writeRegister(SYS_STAT, STAT_UV);
            enableDischarging(1 << ERROR_UVP);
            errorStatus_.bits.UV = 0;
        }
    }
    
    if(errorStatus_.bits.OV) {
        if(data.cellVoltages_[idCellMaxVoltage_] < maxCellVoltage_) {

            if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear OVP error\r\n");
            writeRegister(SYS_STAT, STAT_OV);
            enableCharging(1 << ERROR_OVP);
            errorStatus_.bits.OV = 0;
        }
    }

    if(errorStatus_.bits.SCD) {
        
        if((uint32_t)(mcu::Timer::millis() - data.errorTimestamps_[ERROR_SCD]) > 10UL * 1000UL) {
            if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear SCD error\r\n");
            writeRegister(SYS_STAT, STAT_SCD);
            enableDischarging(1 << ERROR_SCD);
            errorStatus_.bits.SCD = 0;
        }
    }
    
    if(errorStatus_.bits.OCD) {

        if((uint32_t)(mcu::Timer::millis() - data.errorTimestamps_[ERROR_OCD]) > 10UL * 1000UL) {
            if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear OCD error\r\n");
            writeRegister(SYS_STAT, STAT_OCD);
            enableDischarging(1 << ERROR_OCD);
            errorStatus_.bits.OCD = 0;
        }
    }
}

//----------------------------------------------------------------------------
// should be called at least once every 250 ms to get correct coulomb counting

uint8_t bq769x0::update() {
    uint8_t ret = checkStatus(); // does updateCurrent()
    //updateCurrent(); // will only read new current value if alert was triggered
    updateVoltages();
    updateTemperatures();
    updateBalancingSwitches();
    if(ret) { clearErrors(); }
    checkUser();
    return ret;
}

//----------------------------------------------------------------------------
// puts BMS IC into SHIP mode (i.e. switched off)

void bq769x0::shutdown() {
    writeRegister(SYS_CTRL1, 0x0);
    writeRegister(SYS_CTRL1, 0x1);
    writeRegister(SYS_CTRL1, 0x2);
}

//----------------------------------------------------------------------------

bool bq769x0::enableCharging(uint16_t flag) {
    data.chargingDisabled_ &= ~flag;

    if(!data.chargingEnabled_ && !data.chargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000001);  // switch CHG on
        data.chargingEnabled_ = true;

        if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Enabling CHG FET\r\n");

        return true;
    }
    else { return data.chargingEnabled_; }
}

//----------------------------------------------------------------------------

void bq769x0::disableCharging(uint16_t flag) {
    data.chargingDisabled_ |= flag;

    if(data.chargingEnabled_ && data.chargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000001);  // switch CHG off
        data.chargingEnabled_ = false;
        if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Disabling CHG FET\r\n");
    }
}

//----------------------------------------------------------------------------

bool bq769x0::enableDischarging(uint16_t flag) {
    data.dischargingDisabled_ &= ~flag;

    if(!data.dischargingEnabled_ && !data.dischargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000010);  // switch DSG on
        data.dischargingEnabled_ = true;
        if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Enabling DISCHG FET\r\n");
        return true;
    } else { return data.dischargingEnabled_; }
}

//----------------------------------------------------------------------------

void bq769x0::disableDischarging(uint16_t flag) {
    data.dischargingDisabled_ |= flag;
    if(data.dischargingEnabled_ && data.dischargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000010);  // switch DSG off
        data.dischargingEnabled_ = false;
        if(conf.m_Debug) cout << stream::Flags::PGM << PSTR("Disabling DISCHG FET\r\n");
    }
}

//----------------------------------------------------------------------------

// void bq769x0::setAutoBalancing(const bool ab) { conf.autoBalancingEnabled_ = ab; }

//----------------------------------------------------------------------------

// void bq769x0::setBalancingThresholds(uint16_t idleTime_s,
//                                      uint16_t absVoltage_mV,
//                                      uint8_t voltageDifference_mV)
// {
//     conf.balancingMinIdleTime_s_ = idleTime_s;
//     conf.balancingMinCellVoltage_mV_ = absVoltage_mV;
//     conf.balancingMaxVoltageDifference_mV_ = voltageDifference_mV;
// }

//----------------------------------------------------------------------------
// sets balancing registers if balancing is allowed
// (sufficient idle time + voltage)

void bq769x0::updateBalancingSwitches(void) {
    int32_t idleSeconds = (mcu::Timer::millis() - idleTimestamp_) / 1000;
    uint8_t numberOfSections = (NUMBER_OF_CELLS + 4) / 5;

    // check for millis() overflow
    if (idleSeconds < 0) {
        idleTimestamp_ = 0;
        idleSeconds = mcu::Timer::millis() / 1000;
    }

    // check if balancing allowed
    if (conf.autoBalancingEnabled_ && errorStatus_.regByte == 0 &&
        ((conf.balanceCharging_ && charging_ == 2) || idleSeconds >= conf.balancingMinIdleTime_s_) &&
        data.cellVoltages_[idCellMaxVoltage_] > conf.balancingMinCellVoltage_mV_ &&
        (data.cellVoltages_[idCellMaxVoltage_] - data.cellVoltages_[idCellMinVoltage_]) > conf.balancingMaxVoltageDifference_mV_)
    {
        //Serial.println("Balancing enabled!");
        data.balancingStatus_ = 0;  // current status will be set in following loop

        //regCELLBAL_t cellbal;
        uint16_t balancingFlags;
        uint16_t balancingFlagsTarget;

        for (uint8_t section = 0; section < numberOfSections; section++)
        {
            // find cells which should be balanced and sort them by voltage descending
            uint8_t cellList[5];
            uint8_t cellCounter = 0;
            for (uint8_t i = 0; i < 5; i++)
            {
                if (data.cellVoltages_[section*5 + i] < 500)
                    continue;

                if ((data.cellVoltages_[section*5 + i] - data.cellVoltages_[idCellMinVoltage_]) > conf.balancingMaxVoltageDifference_mV_)
                {
                    int j = cellCounter;
                    while (j > 0 && data.cellVoltages_[section*5 + cellList[j - 1]] < data.cellVoltages_[section*5 + i])
                    {
                        cellList[j] = cellList[j - 1];
                        j--;
                    }
                    cellList[j] = i;
                    cellCounter++;
                }
            }

            balancingFlags = 0;
            for (uint8_t i = 0; i < cellCounter; i++)
            {
                // try to enable balancing of current cell
                balancingFlagsTarget = balancingFlags | (1 << cellList[i]);

                // check if attempting to balance adjacent cells
                bool adjacentCellCollision =
                    ((balancingFlagsTarget << 1) & balancingFlags) ||
                    ((balancingFlags << 1) & balancingFlagsTarget);

                if (adjacentCellCollision == false) {
                    balancingFlags = balancingFlagsTarget;
                }
            }


            if(conf.m_Debug) {
                cout << stream::Flags::PGM << PSTR("Setting CELLBAL ") << uint8_t(section+1);
                cout << stream::Flags::PGM << PSTR(" register to: ") << byte2char(balancingFlags) << EOL;
            }


            data.balancingStatus_ |= balancingFlags << section*5;

            // set balancing register for this section
            writeRegister(CELLBAL1+section, balancingFlags);

        } // section loop
    } else if (data.balancingStatus_ > 0) {
        // clear all CELLBAL registers
        for (uint8_t section = 0; section < numberOfSections; section++) {

            if(conf.m_Debug) {
                cout << stream::Flags::PGM << PSTR("Clearing Register CELLBAL ") << uint8_t(section+1) << EOL;
            }

            writeRegister(CELLBAL1+section, 0x0);
        }
        data.balancingStatus_ = 0;
    }
}

//----------------------------------------------------------------------------

// uint16_t bq769x0::getBalancingStatus() { return data.balancingStatus_; }

//----------------------------------------------------------------------------

// void bq769x0::setShuntResistorValue(uint32_t res_uOhm) { conf.shuntResistorValue_uOhm_ = res_uOhm; }

//----------------------------------------------------------------------------

// void bq769x0::setThermistors(uint8_t bitflag) { conf.thermistors_ = bitflag & ((1 << MAX_NUMBER_OF_THERMISTORS) - 1); }

//----------------------------------------------------------------------------

// // void bq769x0::setThermistorBetaValue(uint16_t beta_K) { thermistorBetaValue_ = beta_K; }

//----------------------------------------------------------------------------

// void bq769x0::setBatteryCapacity(int32_t capacity_mAh,
//                                  uint16_t nomVoltage_mV,
//                                  uint16_t fullVoltage_mV)
// {
//     conf.nominalCapacity_ = capacity_mAh * 60 * 60;
//     conf.nominalVoltage_ = nomVoltage_mV;
//     conf.fullVoltage_ = fullVoltage_mV;
// }

//----------------------------------------------------------------------------

void bq769x0::setOCV(uint16_t voltageVsSOC[NUM_OCV_POINTS]) {
    OCV_ = voltageVsSOC;
}

//----------------------------------------------------------------------------

float bq769x0::getSOC(void) {
    return (float) coulombCounter_ / conf.nominalCapacity_ * 100.0;
}

//----------------------------------------------------------------------------
// SOC calculation based on average cell open circuit voltage

void bq769x0::resetSOC(int percent) {
    
    if (percent <= 100 && percent >= 0) {
        coulombCounter_ = (int32_t)(conf.nominalCapacity_ * percent) / 100L;
    } else {  // reset based on OCV

        if(conf.m_Debug) {
            cout << stream::Flags::PGM << PSTR("NumCells: ") << getNumberOfConnectedCells() << 
            stream::Flags::PGM << PSTR(", voltage: ") << data.batVoltage_ << 'V';
        }

        uint16_t voltage = data.batVoltage_ / getNumberOfConnectedCells();

        coulombCounter_ = 0;  // initialize with totally depleted battery (0% SOC)

        for (int i = 0; i < NUM_OCV_POINTS; i++)
        {
            if (OCV_[i] <= voltage) {
                if (i == 0) {
                    coulombCounter_ = conf.nominalCapacity_;  // 100% full
                }
                else {
                    // interpolate between OCV[i] and OCV[i-1]
                    coulombCounter_ = (double) conf.nominalCapacity_ / (NUM_OCV_POINTS - 1.0) *
                    (NUM_OCV_POINTS - 1.0 - i + ((float)voltage - OCV_[i])/(OCV_[i-1] - OCV_[i]));
                }
                return;
            }
        }
    }
}

//----------------------------------------------------------------------------

// void bq769x0::adjADCPackOffset(int16_t offset) { conf.adcPackOffset_ = offset; }

int16_t bq769x0::getADCPackOffset() { return data.adcOffset_ + conf.adcPackOffset_; }

// void bq769x0::adjADCCellsOffset(int16_t offsets[MAX_NUMBER_OF_CELLS]) { conf.adcCellsOffset_ = offsets; }

int16_t bq769x0::getADCCellOffset(uint8_t cell) {
//     if (conf.adcCellsOffset_)
    return data.adcOffset_ + conf.adcCellsOffset_[cell];
//     return data.adcOffset_;
}

//----------------------------------------------------------------------------

// void bq769x0::setTemperatureLimits(int16_t minDischarge_degC,
//                                    int16_t maxDischarge_degC,
//                                    int16_t minCharge_degC,
//                                    int16_t maxCharge_degC)
// {
//     // Temperature limits (°C/10)
//     conf.minCellTempDischarge_ = minDischarge_degC * 10;
//     conf.maxCellTempDischarge_ = maxDischarge_degC * 10;
//     conf.minCellTempCharge_ = minCharge_degC * 10;
//     conf.maxCellTempCharge_ = maxCharge_degC * 10;
// }

//----------------------------------------------------------------------------

// void bq769x0::setIdleCurrentThreshold(uint32_t current_mA) { conf.idleCurrentThreshold_ = current_mA; }

//----------------------------------------------------------------------------

// void bq769x0::setBalanceCharging(bool charging) { conf.balanceCharging_ = charging; }

//----------------------------------------------------------------------------

uint32_t bq769x0::setShortCircuitProtection(uint32_t current_mA, uint16_t delay_us) {
    regPROTECT1_t protect1;
    protect1.bits.RSNS = PROTECT1_RSNS;

    protect1.bits.SCD_THRESH = 0;
    uint8_t temp = (current_mA * conf.shuntResistorValue_uOhm_) / 1000000UL;
    for (uint8_t i = sizeof(SCD_threshold_setting)-1; i > 0; i--) {
        if (temp >= SCD_threshold_setting[i]) {
            protect1.bits.SCD_THRESH = i;
            break;
        }
    }

    protect1.bits.SCD_DELAY = 0;
    for (uint8_t i = sizeof(SCD_delay_setting)-1; i > 0; i--) {
        if (delay_us >= SCD_delay_setting[i]) {
            protect1.bits.SCD_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT1, protect1.regByte);

    // returns the actual current threshold value
    return ((uint32_t)SCD_threshold_setting[protect1.bits.SCD_THRESH] * 1000000UL) /
        conf.shuntResistorValue_uOhm_;
}

//----------------------------------------------------------------------------

// uint32_t bq769x0::setOvercurrentChargeProtection(uint32_t current_mA, uint16_t delay_ms)
// {
//     conf.maxChargeCurrent_ = current_mA;
//     conf.maxChargeCurrent_delay_ = delay_ms;
//     return 0;
// }

//----------------------------------------------------------------------------

uint32_t bq769x0::setOvercurrentDischargeProtection(uint32_t current_mA, uint16_t delay_ms)
{
    regPROTECT2_t protect2;

    protect2.bits.OCD_THRESH = 0;
    uint8_t temp = (current_mA * conf.shuntResistorValue_uOhm_) / 1000000UL;
    for (uint8_t i = sizeof(OCD_threshold_setting)-1; i > 0; i--) {
        if (temp >= OCD_threshold_setting[i]) {
            protect2.bits.OCD_THRESH = i;
            break;
        }
    }

    protect2.bits.OCD_DELAY = 0;
    for (uint8_t i = sizeof(OCD_delay_setting)-1; i > 0; i--) {
        if (delay_ms >= OCD_delay_setting[i]) {
            protect2.bits.OCD_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT2, protect2.regByte);

    // returns the actual current threshold value
    return ((uint32_t)OCD_threshold_setting[protect2.bits.OCD_THRESH] * 1000000UL) /
        conf.shuntResistorValue_uOhm_;
}


//----------------------------------------------------------------------------

uint16_t bq769x0::setCellUndervoltageProtection(uint16_t voltage_mV, uint16_t delay_s)
{
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);

    minCellVoltage_ = voltage_mV;

    uint16_t uv_trip = ((((voltage_mV - data.adcOffset_) * 1000UL) / data.adcGain_) >> 4) & 0x00FF;
    uv_trip += 1;   // always round up for lower cell voltage
    writeRegister(UV_TRIP, uv_trip);

    protect3.bits.UV_DELAY = 0;
    for (uint8_t i = sizeof(UV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= UV_delay_setting[i]) {
            protect3.bits.UV_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT3, protect3.regByte);

    // returns the actual voltage threshold value
    return ((1UL << 12UL | uv_trip << 4) * data.adcGain_) / 1000UL + data.adcOffset_;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::setCellOvervoltageProtection(uint16_t voltage_mV, uint16_t delay_s)
{
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);

    maxCellVoltage_ = voltage_mV;

    uint16_t ov_trip = ((((voltage_mV - data.adcOffset_) * 1000UL) / data.adcGain_) >> 4) & 0x00FF;
    writeRegister(OV_TRIP, ov_trip);

    protect3.bits.OV_DELAY = 0;
    for (uint8_t i = sizeof(OV_delay_setting)-1; i > 0; i--) {
        if (delay_s >= OV_delay_setting[i]) {
            protect3.bits.OV_DELAY = i;
            break;
        }
    }

    writeRegister(PROTECT3, protect3.regByte);

    // returns the actual voltage threshold value
    return ((uint32_t)(1 << 13 | ov_trip << 4) * data.adcGain_) / 1000UL + data.adcOffset_;
}


//----------------------------------------------------------------------------

// int32_t bq769x0::getBatteryCurrent(bool raw) {
//     if (raw) return data.batCurrent_;
//     return data.batCurrent_;
// }
// 
// //----------------------------------------------------------------------------
// 
// uint32_t bq769x0::getBatteryVoltage(bool raw) {
//     if (raw) return data.batVoltage_raw_;
//     return data.batVoltage_;
// }

//----------------------------------------------------------------------------

uint16_t bq769x0::getMaxCellVoltage() { return data.cellVoltages_[idCellMaxVoltage_]; }

//----------------------------------------------------------------------------

uint16_t bq769x0::getMinCellVoltage() { return data.cellVoltages_[idCellMinVoltage_]; }

//----------------------------------------------------------------------------

uint16_t bq769x0::getAvgCellVoltage() { return data.batVoltage_ / getNumberOfConnectedCells(); }

//----------------------------------------------------------------------------

uint16_t bq769x0::getCellVoltage(uint8_t idCell, bool raw) {
    uint8_t i = data.cellIdMap_[idCell];
    if (raw) return data.cellVoltages_raw_[i];
    return data.cellVoltages_[i];
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getCellVoltage_(uint8_t i, bool raw) {
    if (raw) return data.cellVoltages_raw_[i];
    return data.cellVoltages_[i];
}

//----------------------------------------------------------------------------

uint8_t bq769x0::getNumberOfCells(void) {
    return NUMBER_OF_CELLS;
}

//----------------------------------------------------------------------------

uint8_t bq769x0::getNumberOfConnectedCells(void) {
    return data.connectedCells_;
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegC(uint8_t channel) {
    if (channel <= 2) {
        return (float)data.temperatures_[channel] / 10.0;
    } else { return -273.15; }  // Error: Return absolute minimum temperature
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegF(uint8_t channel) {
    return getTemperatureDegC(channel) * 1.8 + 32;
}

//----------------------------------------------------------------------------

int16_t bq769x0::getLowestTemperature() {
    int16_t minTemp = INT16_MAX;
    for(uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++) {
        if(conf.thermistors_ & (1 << i) && data.temperatures_[i] < minTemp) minTemp = data.temperatures_[i];
    }
    return minTemp;
}

int16_t bq769x0::getHighestTemperature() {
    int16_t maxTemp = INT16_MIN;
    for(uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++) {
        if(conf.thermistors_ & (1 << i) && data.temperatures_[i] > maxTemp) maxTemp = data.temperatures_[i];
    }
    return maxTemp;
}

//----------------------------------------------------------------------------

int16_t updateTemperatures_calc(const uint16_t val, const uint16_t beta) {
    // calculate R_thermistor according to bq769x0 datasheet
    uint16_t vtsx = val * 0.382; // mV
    uint32_t rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
    // Temperature calculation using Beta equation
    // - According to bq769x0 datasheet, only 10k thermistors should be used
    // - 25°C reference temperature for Beta equation assumed
    float tmp = 1.0/(1.0/(273.15+25) + 1.0/beta * log(rts/10000.0)); // K
    return (tmp - 273.15) * 10.0;
}


void bq769x0::updateTemperatures() {
    data.temperatures_[0] = updateTemperatures_calc(readDoubleRegister(TS1_HI_BYTE), conf.thermistorBetaValue_[0]);
    
#ifdef IC_BQ76930
    data.temperatures_[1] = updateTemperatures_calc(readDoubleRegister(TS2_HI_BYTE), conf.thermistorBetaValue_[1]);
#endif
    
#ifdef IC_BQ76940
    data.temperatures_[1] = updateTemperatures_calc(readDoubleRegister(TS2_HI_BYTE), conf.thermistorBetaValue_[1]);
    data.temperatures_[1] = updateTemperatures_calc(readDoubleRegister(TS3_HI_BYTE), conf.thermistorBetaValue_[1]);
#endif
}


//----------------------------------------------------------------------------

void bq769x0::updateCurrent()
{
    regSYS_STAT_t sys_stat;
    sys_stat.regByte = readRegister(SYS_STAT);

    // check if new current reading available
    if (sys_stat.bits.CC_READY == 1)
    {
        //Serial.println("reading CC register...");
        data.batCurrent_ = (int16_t)readDoubleRegister(CC_HI_BYTE);
        data.batCurrent_ = ((int32_t)data.batCurrent_ * 8440L) / (int32_t)conf.shuntResistorValue_uOhm_;  // mA

        // is read every 250 ms
        coulombCounter_ += data.batCurrent_ / 4;

        if (coulombCounter_ > conf.nominalCapacity_) {
            coulombCounter_ = conf.nominalCapacity_;
        }
        if (coulombCounter_ < 0) {
            coulombCounter_ = 0;
        }

        if (data.batCurrent_ < 0) {
            coulombCounter2_ += -data.batCurrent_ / 4;
            if (coulombCounter2_ > conf.nominalCapacity_) {
                batCycles_++;
                coulombCounter2_ = 0;
            }
        }

        if (data.batCurrent_ > (int32_t)conf.idleCurrentThreshold_) {
            if (!charging_) {
                charging_ = 1;
                chargeTimestamp_ = mcu::Timer::millis();
            }
            else if (charging_ == 1 && (uint32_t)(mcu::Timer::millis() - chargeTimestamp_) > 60UL * 1000UL) {
                charging_ = 2;
                chargedTimes_++;
            }
        }
        else if (charging_ != 2 || data.batCurrent_ < 10)
            charging_ = 0;

        // reset idleTimestamp
        if (abs(data.batCurrent_) > conf.idleCurrentThreshold_) {
            if(data.batCurrent_ < 0 || !(conf.balanceCharging_ && charging_ == 2))
                idleTimestamp_ = mcu::Timer::millis();
        }

        // no error occured which caused alert
        if (!(sys_stat.regByte & 0b00111111)) {
            data.alertInterruptFlag_ = false;
        }

        writeRegister(SYS_STAT, 0b10000000);  // Clear CC ready flag
    }
}

//----------------------------------------------------------------------------
// reads all cell voltages to array cellVoltages[NUM_CELLS] and updates batVoltage

void bq769x0::updateVoltages() {
    mcu::I2CMaster      Wire;
    uint16_t adcVal = 0;
    uint8_t idCell = 0;
    // read cell voltages
    i2buf[0] = VC1_HI_BYTE;

    // mcu::I2CMaster Wire;
    
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 1);

    idCellMaxVoltage_ = 0;
    idCellMinVoltage_ = 0;
    for (int i = 0; i < NUMBER_OF_CELLS; i++) {
#ifdef BQ769X0_CRC_ENABLED
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 4);
        uint8_t crc;
        uint8_t data_b = i2buf[0];
        adcVal = (data_b & 0b00111111) << 8;

        // CRC of first bytes includes slave address (including R/W bit) and data
        crc = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 1);
        crc = _crc8_ccitt_update(crc, data_b);
        if (crc != i2buf[1]) return; // don't save corrupted value
        data_b = i2buf[2];
        adcVal |= data_b;

        // CRC of subsequent bytes contain only data
        crc = _crc8_ccitt_update(0, data_b);
        if (crc != i2buf[3]) return; // don't save corrupted value
            
#else
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 2);
        // reply
        adcVal = (i2buf[0] & 0b00111111) << 8 | i2buf[1];
#endif

        data.cellVoltages_raw_[i] = adcVal;
        data.cellVoltages_[i] = ((uint32_t)adcVal * data.adcGain_) / 1000 + getADCCellOffset(i);

        if (data.cellVoltages_[i] < 500) { continue; }
        data.cellIdMap_[idCell] = i;
        if (data.cellVoltages_[i] > data.cellVoltages_[idCellMaxVoltage_]) { idCellMaxVoltage_ = i; }
        if (data.cellVoltages_[i] < data.cellVoltages_[idCellMinVoltage_]) { idCellMinVoltage_ = i; }
        idCell++;
    }
    data.connectedCells_ = idCell;
    // read battery pack voltage
    data.batVoltage_raw_ = readDoubleRegister(BAT_HI_BYTE);
    data.batVoltage_ = ((uint32_t)4.0 * data.adcGain_ * data.batVoltage_raw_) / 1000.0 + data.connectedCells_ * getADCPackOffset();

    if(data.batVoltage_ >= data.connectedCells_ * conf.fullVoltage_) {
        if(fullVoltageCount_ == 240) { // 60s * 4(250ms)
            resetSOC(100);
        }
        if(fullVoltageCount_ < 255)
            fullVoltageCount_++;
    } else {
        fullVoltageCount_ = 0;
    }
}

//----------------------------------------------------------------------------

void bq769x0::writeRegister(uint8_t address, uint8_t data) {
    mcu::I2CMaster      Wire;
    i2buf[0] = address;
    i2buf[1] = data;
    // mcu::I2CMaster Wire;
#ifdef BQ769X0_CRC_ENABLED
    // CRC is calculated over the slave address (including R/W bit), register address, and data.
    i2buf[2] = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 0);
    i2buf[2] = _crc8_ccitt_update(i2buf[2], address);
    i2buf[2] = _crc8_ccitt_update(i2buf[2], data);
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 3);
#else
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 2);
#endif
    }


//----------------------------------------------------------------------------

uint8_t bq769x0::readRegister(uint8_t address) {
    mcu::I2CMaster      Wire;
    i2buf[0] = address;
    // mcu::I2CMaster Wire;
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 1);
    uint8_t data;
#ifdef BQ769X0_CRC_ENABLED
    uint8_t wantcrc;
    uint8_t gotcrc;
    do {
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 2);
        data   = i2buf[0];
        gotcrc = i2buf[1];
        // CRC is calculated over the slave address (including R/W bit) and data.
        wantcrc = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 1);
        wantcrc = _crc8_ccitt_update(wantcrc, data);
    } while (gotcrc != wantcrc);
#else
    Wire.read(BQ769X0_I2C_ADDR, i2buf, 1);
    data = i2buf[0];
#endif
    return data;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::readDoubleRegister(uint8_t address) {
    mcu::I2CMaster      Wire;
    i2buf[0] = address;
    // mcu::I2CMaster Wire;
    Wire.write(BQ769X0_I2C_ADDR, i2buf, 1);
    uint16_t result;
#ifdef BQ769X0_CRC_ENABLED
    while(true) {
        Wire.read(BQ769X0_I2C_ADDR, i2buf, 4);
        uint8_t crc;
        uint8_t data = i2buf[0];
        result = (uint16_t)data << 8;
        // CRC of first bytes includes slave address (including R/W bit) and data
        crc = _crc8_ccitt_update(0, (BQ769X0_I2C_ADDR << 1) | 1);
        crc = _crc8_ccitt_update(crc, data);
        if (crc != i2buf[1]) continue;
        data = i2buf[2];
        result |= data;
        // CRC of subsequent bytes contain only data
        crc = _crc8_ccitt_update(0, data);
        if (crc != i2buf[3]) continue;
        break;
    }
#else
    Wire.read(BQ769X0_I2C_ADDR, i2buf, 2);
    result = ((uint16_t)i2buf[0] << 8) | i2buf[1];
#endif
    return result;
}

//----------------------------------------------------------------------------
// The bq769x0 drives the ALERT pin high if the SYS_STAT register contains
// a new value (either new CC reading or an error)

// void bq769x0::setAlertInterruptFlag() {
// // //     interruptTimestamp_ = mcu::Timer::millis();
//     data.alertInterruptFlag_ = true;
// }

//----------------------------------------------------------------------------
// Check custom error conditions like over/under temperature, over charge current

void bq769x0::checkUser()
{
    // charge temperature limits
    if(getLowestTemperature() < conf.minCellTempCharge_ || getHighestTemperature() > conf.maxCellTempCharge_)
    {
        if(!(data.chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP)))
        {
            disableCharging(1 << ERROR_USER_CHG_TEMP);
            errorCounter_[ERROR_USER_CHG_TEMP]++;
            data.errorTimestamps_[ERROR_USER_CHG_TEMP] = mcu::Timer::millis();
        }
    }
    else if(data.chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP))
    {
        enableCharging(1 << ERROR_USER_CHG_TEMP);
    }

    // discharge temperature limits
    if(getLowestTemperature() < conf.minCellTempDischarge_ || getHighestTemperature() > conf.maxCellTempDischarge_)
    {
        if(!(data.dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP)))
        {
            disableDischarging(1 << ERROR_USER_DISCHG_TEMP);
            errorCounter_[ERROR_USER_DISCHG_TEMP]++;
            data.errorTimestamps_[ERROR_USER_DISCHG_TEMP] = mcu::Timer::millis();
        }
    }
    else if(data.dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP))
    {
        enableDischarging(1 << ERROR_USER_DISCHG_TEMP);
    }

    // charge current limit
    // charge current can also come through discharge FET that we can't turn off (regen on P-)
    // that's why this looks a bit funky
    if(data.batCurrent_ > conf.maxChargeCurrent_)
    {
        user_CHGOCD_ReleaseTimestamp_ = 0;

        if(data.chargingEnabled_ && !(data.chargingDisabled_ & (1 << ERROR_USER_CHG_OCD))) {
            if(!user_CHGOCD_TriggerTimestamp_)
                user_CHGOCD_TriggerTimestamp_ = mcu::Timer::millis();
            if((mcu::Timer::millis() - user_CHGOCD_TriggerTimestamp_) > conf.maxChargeCurrent_delay_ || data.user_CHGOCD_ReleasedNow_) {
                disableCharging(1 << ERROR_USER_CHG_OCD);
                errorCounter_[ERROR_USER_CHG_OCD]++;
                data.errorTimestamps_[ERROR_USER_CHG_OCD] = mcu::Timer::millis();
            }
        }
    } else {
        user_CHGOCD_TriggerTimestamp_ = 0;
        data.user_CHGOCD_ReleasedNow_ = false;
        if(data.chargingDisabled_ & (1 << ERROR_USER_CHG_OCD)) {
            if(!user_CHGOCD_ReleaseTimestamp_)
                user_CHGOCD_ReleaseTimestamp_ = mcu::Timer::millis();
            if((uint32_t)(mcu::Timer::millis() - user_CHGOCD_ReleaseTimestamp_) > 10UL * 1000UL) {
                enableCharging(1 << ERROR_USER_CHG_OCD);
                user_CHGOCD_ReleaseTimestamp_ = 0;
                data.user_CHGOCD_ReleasedNow_ = true;
            }
        }
    }
}


void bq769x0::printRegisters() {
    cout << stream::Flags::PGM << PSTR("\r\n0x00  SYS_STAT: ") << byte2char(readRegister(SYS_STAT));
    cout << stream::Flags::PGM << PSTR("\r\n0x01  CELLBAL1: ") << byte2char(readRegister(CELLBAL1));
    cout << stream::Flags::PGM << PSTR("\r\n0x04 SYS_CTRL1: ") << byte2char(readRegister(SYS_CTRL1));
    cout << stream::Flags::PGM << PSTR("\r\n0x05 SYS_CTRL2: ") << byte2char(readRegister(SYS_CTRL2));
    cout << stream::Flags::PGM << PSTR("\r\n0x06  PROTECT1: ") << byte2char(readRegister(PROTECT1));
    cout << stream::Flags::PGM << PSTR("\r\n0x07  PROTECT2: ") << byte2char(readRegister(PROTECT2));
    cout << stream::Flags::PGM << PSTR("\r\n0x08 PROTECT3   ") << byte2char(readRegister(PROTECT3));
    cout << stream::Flags::PGM << PSTR("\r\n0x09 OV_TRIP:   ") << byte2char(readRegister(OV_TRIP));
    cout << stream::Flags::PGM << PSTR("\r\n0x0A UV_TRIP:   ") << byte2char(readRegister(UV_TRIP));
    cout << stream::Flags::PGM << PSTR("\r\n0x0B CC_CFG:    ") << byte2char(readRegister(CC_CFG));
    cout << stream::Flags::PGM << PSTR("\r\n0x32 CC_HI_LO:  ") << readDoubleRegister(CC_HI_BYTE);
    cout << stream::Flags::PGM << PSTR("\r\n0x2A BAT_HI_LO: ") << readDoubleRegister(BAT_HI_BYTE);
    cout << stream::Flags::PGM << PSTR("\r\nADCGAIN:        ") << data.adcGain_;
    cout << stream::Flags::PGM << PSTR("\r\nADCOFFSET:      ") << (uint16_t)data.adcOffset_;
    cout << stream::Flags::PGM << PSTR("\r\nCHG DIS:        ") << data.chargingDisabled_;
    cout << stream::Flags::PGM << PSTR("\r\nDISCHG DIS:     ") << data.dischargingDisabled_ << "\r\n";
}



}
