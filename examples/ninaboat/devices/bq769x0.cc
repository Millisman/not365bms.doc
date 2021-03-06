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
// #define INT16_MAX 0x7fffL
// #define INT16_MIN (-INT16_MAX - 1L)

namespace devices {

// TODO move this
const char *byte2char(int x)
{
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}

// TODO move this
uint8_t _crc8_ccitt_update (uint8_t inCrc, uint8_t inData)
{
    uint8_t data = inCrc ^ inData;

    for (uint8_t i = 0; i < 8; i++)
    {
        if ((data & 0x80) != 0)
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
    }
    return data;
}

// uint8_t batCycles_;
// uint8_t chargedTimes_;
// uint8_t errorCounter_[NUM_ERRORS];
bq769x0::bq769x0(BqIC bqType, uint8_t bqI2CAddress, bool crc):
    cout                    (mcu::Usart::get()),
    m_Debug                 (false),
    chargingEnabled_        (false),
    dischargingEnabled_     (false),
    crcEnabled_(crc),
    alertInterruptFlag_     (true),
    balanceCharging_        (false),
    autoBalancingEnabled_   (false),
    user_CHGOCD_ReleasedNow_(false),
    I2CAddress_             (bqI2CAddress),
    type_                   (bqType),
    chargingDisabled_       (0),
    dischargingDisabled_    (0),
    shuntResistorValue_uOhm_(1000), // 1mOhm
    thermistorBetaValue_    (3435), // typical value for Semitec 103AT-5 thermistor: 3435
    OCV_                    (0), // Open Circuit Voltage of cell for SOC 100%, 95%, ..., 5%, 0%
    numberOfCells_          (15), // number of cells allowed by IC (MAX)
    connectedCells_         (0), // actual number of cells connected
    idCellMaxVoltage_       (0),
    idCellMinVoltage_       (0),
    batVoltage_             (0), // mV
    batVoltage_raw_         (0), // adc val
    batCurrent_             (0), // mA
    batCurrent_raw_         (0), // adc val
    thermistors_            (0),
    nominalVoltage_         (3600), // mV, nominal voltage of single cell in battery pack
    fullVoltage_            (4200), // mV, full voltage of single cell in battery pack
    nominalCapacity_        (0), // mAs, nominal capacity of battery pack, max. 580 Ah possible @ 3.7V
    coulombCounter_         (0), // mAs (= milli Coulombs) for current integration
    coulombCounter2_        (0), // mAs (= milli Coulombs) for tracking battery cycles
    maxChargeCurrent_       (0), // Current limits (mA)
    maxChargeCurrent_delay_ (8), // Current limits
    maxDischargeCurrent_    (0), // Current limits (mA)
    idleCurrentThreshold_   (30), // Current (mA)
    minCellTempCharge_      (0), // Temperature limits (°C/10)
    minCellTempDischarge_   (0), // Temperature limits (°C/10)
    maxCellTempCharge_      (0), // Temperature limits (°C/10)
    maxCellTempDischarge_   (0), // Temperature limits (°C/10)
    maxCellVoltage_         (0), // Cell voltage limits (mV)
    minCellVoltage_         (0), // Cell voltage limits (mV)
    balancingMinCellVoltage_mV_(3400), // Cell voltage (mV)
    balancingMaxVoltageDifference_mV_ (20), // Cell voltage (mV)
    adcGain_                (0), // uV/LSB
    adcOffset_              (0), // mV
    adcPackOffset_          (0), // mV
    adcCellsOffset_         (0), // mV
    balancingStatus_        (0), // holds on/off status of balancing switches
    balancingMinIdleTime_s_ (1800),
    idleTimestamp_          (0),
    charging_               (0),
    chargeTimestamp_        (0),
    fullVoltageCount_       (0),
    user_CHGOCD_TriggerTimestamp_ (0),
    user_CHGOCD_ReleaseTimestamp_ (0),
    interruptTimestamp_     (0)
{
    memset(errorTimestamps_,    0, sizeof(errorTimestamps_));
    memset(temperatures_,       0, sizeof(temperatures_));
    memset(cellVoltages_,       0, sizeof(cellVoltages_));
    memset(cellVoltages_raw_,   0, sizeof(cellVoltages_raw_));
    memset(cellIdMap_,          0, sizeof(cellIdMap_));

    if (type_ == BqIC::bq76920) {
        numberOfCells_ = 5;
        setThermistors(0b001);
    } else if (type_ == BqIC::bq76930) {
        numberOfCells_ = 10;
        setThermistors(0b011);
    } else {
        setThermistors(0b111);
    }

    while (true) {
        // test communication
        writeRegister(CC_CFG, 0x19);       // should be set to 0x19 according to datasheet
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
    adcOffset_ = readRegister(ADCOFFSET);
    adcGain_ = 365 + (
        ((readRegister(ADCGAIN1) & 0b00001100) << 1) |
        ((readRegister(ADCGAIN2) & 0b11100000) >> 5)); // uV/LSB
        
}

//-----------------------------------------------------------------------------

void bq769x0::setDebug(bool debug) { m_Debug = debug; }

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

    if (alertInterruptFlag_ || errorStatus_.regByte) {
        regSYS_STAT_t sys_stat;
        sys_stat.regByte = readRegister(SYS_STAT);

        // first check, if only a new CC reading is available
        if (sys_stat.bits.CC_READY == 1) {
            if(m_Debug) cout << stream::Flags::PGM << PSTR("Interrupt: CC ready\r\n");
            updateCurrent();  // automatically clears CC ready flag
        }

        // Serious error occured
        if (sys_stat.regByte & STAT_FLAGS) {
            if (!errorStatus_.bits.DEVICE_XREADY && sys_stat.bits.DEVICE_XREADY) { // XR error
                chargingEnabled_ = dischargingEnabled_ = false;
                chargingDisabled_ |= (1 << ERROR_XREADY);
                dischargingDisabled_ |= (1 << ERROR_XREADY);
                errorCounter_[ERROR_XREADY]++;
                errorTimestamps_[ERROR_XREADY] = mcu::Timer::millis();
                if(m_Debug) cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: XREADY\r\n");
            }
            if (!errorStatus_.bits.OVRD_ALERT && sys_stat.bits.OVRD_ALERT) { // Alert error
                chargingEnabled_ = dischargingEnabled_ = false;
                chargingDisabled_ |= (1 << ERROR_ALERT);
                dischargingDisabled_ |= (1 << ERROR_ALERT);

                errorCounter_[ERROR_ALERT]++;
                errorTimestamps_[ERROR_ALERT] = mcu::Timer::millis();

                if(m_Debug) cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: ALERT\r\n");

            }
            if (sys_stat.bits.UV) { // UV error
                dischargingEnabled_ = false;
                dischargingDisabled_ |= (1 << ERROR_UVP);

                errorCounter_[ERROR_UVP]++;
                errorTimestamps_[ERROR_UVP] = mcu::Timer::millis();

                if(m_Debug)
                    cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: UVP\r\n");

            }
            if (sys_stat.bits.OV) { // OV error
                chargingEnabled_ = false;
                chargingDisabled_ |= (1 << ERROR_OVP);

                errorCounter_[ERROR_OVP]++;
                errorTimestamps_[ERROR_OVP] = mcu::Timer::millis();

                if(m_Debug)
                    cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: OVP\r\n");

            }
            if (sys_stat.bits.SCD) { // SCD
                dischargingEnabled_ = false;
                dischargingDisabled_ |= (1 << ERROR_SCD);

                errorCounter_[ERROR_SCD]++;
                errorTimestamps_[ERROR_SCD] = mcu::Timer::millis();

                if(m_Debug)
                    cout << stream::Flags::PGM << PSTR("bq769x0 ERROR: SCD\r\n");


            }
            if (sys_stat.bits.OCD) { // OCD
                dischargingEnabled_ = false;
                dischargingDisabled_ |= (1 << ERROR_OCD);

                errorCounter_[ERROR_OCD]++;
                errorTimestamps_[ERROR_OCD] = mcu::Timer::millis();

                if(m_Debug)
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
        if((uint32_t)(mcu::Timer::millis() - errorTimestamps_[ERROR_XREADY]) > 3UL * 1000UL) {
            if(m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear XREADY error\r\n");
            writeRegister(SYS_STAT, STAT_DEVICE_XREADY);
            enableCharging(1 << ERROR_XREADY);
            enableDischarging(1 << ERROR_XREADY);
            errorStatus_.bits.DEVICE_XREADY = 0;
        }
    }
    
    if(errorStatus_.bits.OVRD_ALERT) {
        if(m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear ALERT error\r\n");
        writeRegister(SYS_STAT, STAT_OVRD_ALERT);
        enableCharging(1 << ERROR_ALERT);
        enableDischarging(1 << ERROR_ALERT);
        errorStatus_.bits.OVRD_ALERT = 0;
    }
    
    if(errorStatus_.bits.UV) {
        if(cellVoltages_[idCellMinVoltage_] > minCellVoltage_) {
            if(m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear UVP error\r\n");
            writeRegister(SYS_STAT, STAT_UV);
            enableDischarging(1 << ERROR_UVP);
            errorStatus_.bits.UV = 0;
        }
    }
    
    if(errorStatus_.bits.OV) {
        if(cellVoltages_[idCellMaxVoltage_] < maxCellVoltage_) {

            if(m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear OVP error\r\n");
            writeRegister(SYS_STAT, STAT_OV);
            enableCharging(1 << ERROR_OVP);
            errorStatus_.bits.OV = 0;
        }
    }

    if(errorStatus_.bits.SCD) {
        
        if((uint32_t)(mcu::Timer::millis() - errorTimestamps_[ERROR_SCD]) > 10UL * 1000UL) {
            if(m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear SCD error\r\n");
            writeRegister(SYS_STAT, STAT_SCD);
            enableDischarging(1 << ERROR_SCD);
            errorStatus_.bits.SCD = 0;
        }
    }
    
    if(errorStatus_.bits.OCD) {

        if((uint32_t)(mcu::Timer::millis() - errorTimestamps_[ERROR_OCD]) > 10UL * 1000UL) {
            if(m_Debug) cout << stream::Flags::PGM << PSTR("Attempting to clear OCD error\r\n");
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
    chargingDisabled_ &= ~flag;

    if(!chargingEnabled_ && !chargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000001);  // switch CHG on
        chargingEnabled_ = true;

        if(m_Debug) cout << stream::Flags::PGM << PSTR("Enabling CHG FET\r\n");

        return true;
    }
    else { return chargingEnabled_; }
}

//----------------------------------------------------------------------------

void bq769x0::disableCharging(uint16_t flag) {
    chargingDisabled_ |= flag;

    if(chargingEnabled_ && chargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000001);  // switch CHG off
        chargingEnabled_ = false;
        if(m_Debug) cout << stream::Flags::PGM << PSTR("Disabling CHG FET\r\n");
    }
}

//----------------------------------------------------------------------------

bool bq769x0::enableDischarging(uint16_t flag) {
    dischargingDisabled_ &= ~flag;

    if(!dischargingEnabled_ && !dischargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 | 0b00000010);  // switch DSG on
        dischargingEnabled_ = true;
        if(m_Debug) cout << stream::Flags::PGM << PSTR("Enabling DISCHG FET\r\n");
        return true;
    } else { return dischargingEnabled_; }
}

//----------------------------------------------------------------------------

void bq769x0::disableDischarging(uint16_t flag) {
    dischargingDisabled_ |= flag;
    if(dischargingEnabled_ && dischargingDisabled_) {
        int sys_ctrl2;
        sys_ctrl2 = readRegister(SYS_CTRL2);
        writeRegister(SYS_CTRL2, sys_ctrl2 & ~0b00000010);  // switch DSG off
        dischargingEnabled_ = false;
        if(m_Debug) cout << stream::Flags::PGM << PSTR("Disabling DISCHG FET\r\n");
    }
}

//----------------------------------------------------------------------------

void bq769x0::setAutoBalancing(const bool ab) { autoBalancingEnabled_ = ab; }

//----------------------------------------------------------------------------

void bq769x0::setBalancingThresholds(uint16_t idleTime_s,
                                     uint16_t absVoltage_mV,
                                     uint8_t voltageDifference_mV)
{
    balancingMinIdleTime_s_ = idleTime_s;
    balancingMinCellVoltage_mV_ = absVoltage_mV;
    balancingMaxVoltageDifference_mV_ = voltageDifference_mV;
}

//----------------------------------------------------------------------------
// sets balancing registers if balancing is allowed
// (sufficient idle time + voltage)

void bq769x0::updateBalancingSwitches(void) {
    int32_t idleSeconds = (mcu::Timer::millis() - idleTimestamp_) / 1000;
    uint8_t numberOfSections = (numberOfCells_ + 4) / 5;

    // check for millis() overflow
    if (idleSeconds < 0) {
        idleTimestamp_ = 0;
        idleSeconds = mcu::Timer::millis() / 1000;
    }

    // check if balancing allowed
    if (autoBalancingEnabled_ && errorStatus_.regByte == 0 &&
        ((balanceCharging_ && charging_ == 2) || idleSeconds >= balancingMinIdleTime_s_) &&
        cellVoltages_[idCellMaxVoltage_] > balancingMinCellVoltage_mV_ &&
        (cellVoltages_[idCellMaxVoltage_] - cellVoltages_[idCellMinVoltage_]) > balancingMaxVoltageDifference_mV_)
    {
        //Serial.println("Balancing enabled!");
        balancingStatus_ = 0;  // current status will be set in following loop

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
                if (cellVoltages_[section*5 + i] < 500)
                    continue;

                if ((cellVoltages_[section*5 + i] - cellVoltages_[idCellMinVoltage_]) > balancingMaxVoltageDifference_mV_)
                {
                    int j = cellCounter;
                    while (j > 0 && cellVoltages_[section*5 + cellList[j - 1]] < cellVoltages_[section*5 + i])
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


            if(m_Debug) {
                cout << stream::Flags::PGM << PSTR("Setting CELLBAL ") << uint8_t(section+1);
                cout << stream::Flags::PGM << PSTR(" register to: ") << byte2char(balancingFlags) << EOL;
            }


            balancingStatus_ |= balancingFlags << section*5;

            // set balancing register for this section
            writeRegister(CELLBAL1+section, balancingFlags);

        } // section loop
    } else if (balancingStatus_ > 0) {
        // clear all CELLBAL registers
        for (uint8_t section = 0; section < numberOfSections; section++) {

            if(m_Debug) {
                cout << stream::Flags::PGM << PSTR("Clearing Register CELLBAL ") << uint8_t(section+1) << EOL;
            }

            writeRegister(CELLBAL1+section, 0x0);
        }
        balancingStatus_ = 0;
    }
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getBalancingStatus() { return balancingStatus_; }

//----------------------------------------------------------------------------

void bq769x0::setShuntResistorValue(uint32_t res_uOhm) { shuntResistorValue_uOhm_ = res_uOhm; }

//----------------------------------------------------------------------------

void bq769x0::setThermistors(uint8_t bitflag) { thermistors_ = bitflag & ((1 << MAX_NUMBER_OF_THERMISTORS) - 1); }

//----------------------------------------------------------------------------

void bq769x0::setThermistorBetaValue(uint16_t beta_K) { thermistorBetaValue_ = beta_K; }

//----------------------------------------------------------------------------

void bq769x0::setBatteryCapacity(int32_t capacity_mAh,
                                 uint16_t nomVoltage_mV,
                                 uint16_t fullVoltage_mV)
{
    nominalCapacity_ = capacity_mAh * 60 * 60;
    nominalVoltage_ = nomVoltage_mV;
    fullVoltage_ = fullVoltage_mV;
}

//----------------------------------------------------------------------------

void bq769x0::setOCV(uint16_t voltageVsSOC[NUM_OCV_POINTS]) {
    OCV_ = voltageVsSOC;
}

//----------------------------------------------------------------------------

float bq769x0::getSOC(void) {
    return (float) coulombCounter_ / nominalCapacity_ * 100.0;
}

//----------------------------------------------------------------------------
// SOC calculation based on average cell open circuit voltage

void bq769x0::resetSOC(int percent) {
    
    if (percent <= 100 && percent >= 0) {
        coulombCounter_ = (int32_t)(nominalCapacity_ * percent) / 100L;
    } else {  // reset based on OCV

        if(m_Debug) {
            cout << stream::Flags::PGM << PSTR("NumCells: ") << getNumberOfConnectedCells() << 
                stream::Flags::PGM << PSTR(", voltage: ") << getBatteryVoltage() << 'V';
        }

        uint16_t voltage = getBatteryVoltage() / getNumberOfConnectedCells();

        coulombCounter_ = 0;  // initialize with totally depleted battery (0% SOC)

        for (int i = 0; i < NUM_OCV_POINTS; i++)
        {
            if (OCV_[i] <= voltage) {
                if (i == 0) {
                    coulombCounter_ = nominalCapacity_;  // 100% full
                }
                else {
                    // interpolate between OCV[i] and OCV[i-1]
                    coulombCounter_ = (double) nominalCapacity_ / (NUM_OCV_POINTS - 1.0) *
                    (NUM_OCV_POINTS - 1.0 - i + ((float)voltage - OCV_[i])/(OCV_[i-1] - OCV_[i]));
                }
                return;
            }
        }
    }
}

//----------------------------------------------------------------------------

void bq769x0::adjADCPackOffset(int16_t offset) { adcPackOffset_ = offset; }

int16_t bq769x0::getADCPackOffset() { return adcOffset_ + adcPackOffset_; }

void bq769x0::adjADCCellsOffset(int16_t offsets[MAX_NUMBER_OF_CELLS]) { adcCellsOffset_ = offsets; }

int16_t bq769x0::getADCCellOffset(uint8_t cell) {
    if(adcCellsOffset_) return adcOffset_ + adcCellsOffset_[cell];
    return adcOffset_;
}

//----------------------------------------------------------------------------

void bq769x0::setTemperatureLimits(int16_t minDischarge_degC,
                                   int16_t maxDischarge_degC,
                                   int16_t minCharge_degC,
                                   int16_t maxCharge_degC)
{
    // Temperature limits (°C/10)
    minCellTempDischarge_ = minDischarge_degC * 10;
    maxCellTempDischarge_ = maxDischarge_degC * 10;
    minCellTempCharge_ = minCharge_degC * 10;
    maxCellTempCharge_ = maxCharge_degC * 10;
}

//----------------------------------------------------------------------------

void bq769x0::setIdleCurrentThreshold(uint32_t current_mA) { idleCurrentThreshold_ = current_mA; }

//----------------------------------------------------------------------------

void bq769x0::setBalanceCharging(bool charging) { balanceCharging_ = charging; }

//----------------------------------------------------------------------------

uint32_t bq769x0::setShortCircuitProtection(uint32_t current_mA, uint16_t delay_us) {
    regPROTECT1_t protect1;
    protect1.bits.RSNS = PROTECT1_RSNS;

    protect1.bits.SCD_THRESH = 0;
    uint8_t temp = (current_mA * shuntResistorValue_uOhm_) / 1000000UL;
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
        shuntResistorValue_uOhm_;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::setOvercurrentChargeProtection(uint32_t current_mA, uint16_t delay_ms)
{
    maxChargeCurrent_ = current_mA;
    maxChargeCurrent_delay_ = delay_ms;
    return 0;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::setOvercurrentDischargeProtection(uint32_t current_mA, uint16_t delay_ms)
{
    regPROTECT2_t protect2;

    protect2.bits.OCD_THRESH = 0;
    uint8_t temp = (current_mA * shuntResistorValue_uOhm_) / 1000000UL;
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
        shuntResistorValue_uOhm_;
}


//----------------------------------------------------------------------------

uint16_t bq769x0::setCellUndervoltageProtection(uint16_t voltage_mV, uint16_t delay_s)
{
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);

    minCellVoltage_ = voltage_mV;

    uint16_t uv_trip = ((((voltage_mV - adcOffset_) * 1000UL) / adcGain_) >> 4) & 0x00FF;
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
    return ((1UL << 12UL | uv_trip << 4) * adcGain_) / 1000UL + adcOffset_;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::setCellOvervoltageProtection(uint16_t voltage_mV, uint16_t delay_s)
{
    regPROTECT3_t protect3;
    protect3.regByte = readRegister(PROTECT3);

    maxCellVoltage_ = voltage_mV;

    uint16_t ov_trip = ((((voltage_mV - adcOffset_) * 1000UL) / adcGain_) >> 4) & 0x00FF;
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
    return ((uint32_t)(1 << 13 | ov_trip << 4) * adcGain_) / 1000UL + adcOffset_;
}


//----------------------------------------------------------------------------

int32_t bq769x0::getBatteryCurrent(bool raw) {
    if (raw) return batCurrent_raw_;
    return batCurrent_;
}

//----------------------------------------------------------------------------

uint32_t bq769x0::getBatteryVoltage(bool raw) {
    if (raw) return batVoltage_raw_;
    return batVoltage_;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getMaxCellVoltage() { return cellVoltages_[idCellMaxVoltage_]; }

//----------------------------------------------------------------------------

uint16_t bq769x0::getMinCellVoltage() { return cellVoltages_[idCellMinVoltage_]; }

//----------------------------------------------------------------------------

uint16_t bq769x0::getAvgCellVoltage() { return getBatteryVoltage() / getNumberOfConnectedCells(); }

//----------------------------------------------------------------------------

uint16_t bq769x0::getCellVoltage(uint8_t idCell, bool raw) {
    uint8_t i = cellIdMap_[idCell];
    if (raw) return cellVoltages_raw_[i];
    return cellVoltages_[i];
}

//----------------------------------------------------------------------------

uint16_t bq769x0::getCellVoltage_(uint8_t i, bool raw) {
    if (raw) return cellVoltages_raw_[i];
    return cellVoltages_[i];
}

//----------------------------------------------------------------------------

uint8_t bq769x0::getNumberOfCells(void) {
    return numberOfCells_;
}

//----------------------------------------------------------------------------

uint8_t bq769x0::getNumberOfConnectedCells(void) {
    return connectedCells_;
}

//----------------------------------------------------------------------------

float bq769x0::getTemperatureDegC(uint8_t channel) {
    if (channel <= 2) {
        return (float)temperatures_[channel] / 10.0;
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
        if(thermistors_ & (1 << i) && temperatures_[i] < minTemp) minTemp = temperatures_[i];
    }
    return minTemp;
}

int16_t bq769x0::getHighestTemperature() {
    int16_t maxTemp = INT16_MIN;
    for(uint8_t i = 0; i < MAX_NUMBER_OF_THERMISTORS; i++) {
        if(thermistors_ & (1 << i) && temperatures_[i] > maxTemp) maxTemp = temperatures_[i];
    }
    return maxTemp;
}

//----------------------------------------------------------------------------

void bq769x0::updateTemperatures() {
    float tmp = 0;
    int adcVal = 0;
    int vtsx = 0;
    uint32_t rts = 0;

    // calculate R_thermistor according to bq769x0 datasheet
    adcVal = readDoubleRegister(TS1_HI_BYTE);
    vtsx = adcVal * 0.382; // mV
    rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm

    // Temperature calculation using Beta equation
    // - According to bq769x0 datasheet, only 10k thermistors should be used
    // - 25°C reference temperature for Beta equation assumed
    tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue_*log(rts/10000.0)); // K
    temperatures_[0] = (tmp - 273.15) * 10.0;

    if (type_ == bq76930 || type_ == bq76940) {
        adcVal = readDoubleRegister(TS2_HI_BYTE);
        vtsx = adcVal * 0.382; // mV
        rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
        tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue_*log(rts/10000.0)); // K
        temperatures_[1] = (tmp - 273.15) * 10.0;
    }

    if (type_ == bq76940) {
        adcVal = readDoubleRegister(TS3_HI_BYTE);
        vtsx = adcVal * 0.382; // mV
        rts = 10000.0 * vtsx / (3300.0 - vtsx); // Ohm
        tmp = 1.0/(1.0/(273.15+25) + 1.0/thermistorBetaValue_*log(rts/10000.0)); // K
        temperatures_[2] = (tmp - 273.15) * 10.0;
    }
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
        batCurrent_raw_ = (int16_t)readDoubleRegister(CC_HI_BYTE);
        batCurrent_ = ((int32_t)batCurrent_raw_ * 8440L) / (int32_t)shuntResistorValue_uOhm_;  // mA

        // is read every 250 ms
        coulombCounter_ += batCurrent_ / 4;

        if (coulombCounter_ > nominalCapacity_) {
            coulombCounter_ = nominalCapacity_;
        }
        if (coulombCounter_ < 0) {
            coulombCounter_ = 0;
        }

        if (batCurrent_ < 0) {
            coulombCounter2_ += -batCurrent_ / 4;
            if (coulombCounter2_ > nominalCapacity_) {
                batCycles_++;
                coulombCounter2_ = 0;
            }
        }

        if (batCurrent_ > (int32_t)idleCurrentThreshold_) {
            if (!charging_) {
                charging_ = 1;
                chargeTimestamp_ = mcu::Timer::millis();
            }
            else if (charging_ == 1 && (uint32_t)(mcu::Timer::millis() - chargeTimestamp_) > 60UL * 1000UL) {
                charging_ = 2;
                chargedTimes_++;
            }
        }
        else if (charging_ != 2 || batCurrent_ < 10)
            charging_ = 0;

        // reset idleTimestamp
        if (abs(batCurrent_) > idleCurrentThreshold_) {
            if(batCurrent_ < 0 || !(balanceCharging_ && charging_ == 2))
                idleTimestamp_ = mcu::Timer::millis();
        }

        // no error occured which caused alert
        if (!(sys_stat.regByte & 0b00111111)) {
            alertInterruptFlag_ = false;
        }

        writeRegister(SYS_STAT, 0b10000000);  // Clear CC ready flag
    }
}

//----------------------------------------------------------------------------
// reads all cell voltages to array cellVoltages[NUM_CELLS] and updates batVoltage

void bq769x0::updateVoltages()
{
    uint16_t adcVal = 0;
    uint8_t idCell = 0;
    // read cell voltages
    i2buf[0] = VC1_HI_BYTE;

    mcu::I2CMaster Wire;
    
    Wire.write(I2CAddress_, i2buf, 1);

    idCellMaxVoltage_ = 0;
    idCellMinVoltage_ = 0;
    for (int i = 0; i < numberOfCells_; i++) {
        if (crcEnabled_ == true) {
            Wire.read(I2CAddress_, i2buf, 4);
            uint8_t crc;
            uint8_t data = i2buf[0];
            adcVal = (data & 0b00111111) << 8;

            // CRC of first bytes includes slave address (including R/W bit) and data
            crc = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 1);
            crc = _crc8_ccitt_update(crc, data);
            if (crc != i2buf[1]) return; // don't save corrupted value
            data = i2buf[2];
            adcVal |= data;

            // CRC of subsequent bytes contain only data
            crc = _crc8_ccitt_update(0, data);
            if (crc != i2buf[3]) return; // don't save corrupted value
            
        } else {
            Wire.read(I2CAddress_, i2buf, 2);
            // reply
            adcVal = (i2buf[0] & 0b00111111) << 8 | i2buf[1];
        }

        cellVoltages_raw_[i] = adcVal;
        cellVoltages_[i] = ((uint32_t)adcVal * adcGain_) / 1000 + getADCCellOffset(i);

        if (cellVoltages_[i] < 500) { continue; }
        cellIdMap_[idCell] = i;
        if (cellVoltages_[i] > cellVoltages_[idCellMaxVoltage_]) { idCellMaxVoltage_ = i; }
        if (cellVoltages_[i] < cellVoltages_[idCellMinVoltage_]) { idCellMinVoltage_ = i; }
        idCell++;
    }
    connectedCells_ = idCell;
    // read battery pack voltage
    batVoltage_raw_ = readDoubleRegister(BAT_HI_BYTE);
    batVoltage_ = ((uint32_t)4.0 * adcGain_ * batVoltage_raw_) / 1000.0 + connectedCells_ * getADCPackOffset();

    if(batVoltage_ >= connectedCells_ * fullVoltage_) {
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
    i2buf[0] = address;
    i2buf[1] = data;
    mcu::I2CMaster Wire;
    if (crcEnabled_ == true) {
        // CRC is calculated over the slave address (including R/W bit), register address, and data.
        i2buf[2] = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 0);
        i2buf[2] = _crc8_ccitt_update(i2buf[2], address);
        i2buf[2] = _crc8_ccitt_update(i2buf[2], data);
        Wire.write(I2CAddress_, i2buf, 3);
    } else {
        Wire.write(I2CAddress_, i2buf, 2);
    }
}

//----------------------------------------------------------------------------

uint8_t bq769x0::readRegister(uint8_t address) {
    i2buf[0] = address;
    mcu::I2CMaster Wire;
    Wire.write(I2CAddress_, i2buf, 1);
    uint8_t data;
    if (crcEnabled_ == true) {
        uint8_t wantcrc;
        uint8_t gotcrc;
        do {
            Wire.read(I2CAddress_, i2buf, 2);
            data   = i2buf[0];
            gotcrc = i2buf[1];
            // CRC is calculated over the slave address (including R/W bit) and data.
            wantcrc = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 1);
            wantcrc = _crc8_ccitt_update(wantcrc, data);
        } while (gotcrc != wantcrc);
    }
    else {
        Wire.read(I2CAddress_, i2buf, 1);
        data = i2buf[0];
    }

    return data;
}

//----------------------------------------------------------------------------

uint16_t bq769x0::readDoubleRegister(uint8_t address) {
    i2buf[0] = address;
    mcu::I2CMaster Wire;
    Wire.write(I2CAddress_, i2buf, 1);
    uint16_t result;
    if (crcEnabled_ == true) {
        while(true) {
            Wire.read(I2CAddress_, i2buf, 4);
            uint8_t crc;
            uint8_t data = i2buf[0];
            result = (uint16_t)data << 8;
            // CRC of first bytes includes slave address (including R/W bit) and data
            crc = _crc8_ccitt_update(0, (I2CAddress_ << 1) | 1);
            crc = _crc8_ccitt_update(crc, data);
            if (crc != i2buf[1]) continue;
            data = i2buf[2];
            result |= data;
            // CRC of subsequent bytes contain only data
            crc = _crc8_ccitt_update(0, data);
            if (crc != i2buf[3]) continue;
            break;
        }
    } else {
        Wire.read(I2CAddress_, i2buf, 2);
        result = ((uint16_t)i2buf[0] << 8) | i2buf[1];
    }
    return result;
}

//----------------------------------------------------------------------------
// The bq769x0 drives the ALERT pin high if the SYS_STAT register contains
// a new value (either new CC reading or an error)

void bq769x0::setAlertInterruptFlag()
{
    interruptTimestamp_ = mcu::Timer::millis();
    alertInterruptFlag_ = true;
}

//----------------------------------------------------------------------------
// Check custom error conditions like over/under temperature, over charge current

void bq769x0::checkUser()
{
    // charge temperature limits
    if(getLowestTemperature() < minCellTempCharge_ || getHighestTemperature() > maxCellTempCharge_)
    {
        if(!(chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP)))
        {
            disableCharging(1 << ERROR_USER_CHG_TEMP);
            errorCounter_[ERROR_USER_CHG_TEMP]++;
            errorTimestamps_[ERROR_USER_CHG_TEMP] = mcu::Timer::millis();
        }
    }
    else if(chargingDisabled_ & (1 << ERROR_USER_CHG_TEMP))
    {
        enableCharging(1 << ERROR_USER_CHG_TEMP);
    }

    // discharge temperature limits
    if(getLowestTemperature() < minCellTempDischarge_ || getHighestTemperature() > maxCellTempDischarge_)
    {
        if(!(dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP)))
        {
            disableDischarging(1 << ERROR_USER_DISCHG_TEMP);
            errorCounter_[ERROR_USER_DISCHG_TEMP]++;
            errorTimestamps_[ERROR_USER_DISCHG_TEMP] = mcu::Timer::millis();
        }
    }
    else if(dischargingDisabled_ & (1 << ERROR_USER_DISCHG_TEMP))
    {
        enableDischarging(1 << ERROR_USER_DISCHG_TEMP);
    }

    // charge current limit
    // charge current can also come through discharge FET that we can't turn off (regen on P-)
    // that's why this looks a bit funky
    if(batCurrent_ > maxChargeCurrent_)
    {
        user_CHGOCD_ReleaseTimestamp_ = 0;

        if(chargingEnabled_ && !(chargingDisabled_ & (1 << ERROR_USER_CHG_OCD)))
        {
            if(!user_CHGOCD_TriggerTimestamp_)
                user_CHGOCD_TriggerTimestamp_ = mcu::Timer::millis();

            if((mcu::Timer::millis() - user_CHGOCD_TriggerTimestamp_) > maxChargeCurrent_delay_ || user_CHGOCD_ReleasedNow_)
            {
                disableCharging(1 << ERROR_USER_CHG_OCD);
                errorCounter_[ERROR_USER_CHG_OCD]++;
                errorTimestamps_[ERROR_USER_CHG_OCD] = mcu::Timer::millis();
            }
        }
    }
    else
    {
        user_CHGOCD_TriggerTimestamp_ = 0;
        user_CHGOCD_ReleasedNow_ = false;

        if(chargingDisabled_ & (1 << ERROR_USER_CHG_OCD))
        {
            if(!user_CHGOCD_ReleaseTimestamp_)
                user_CHGOCD_ReleaseTimestamp_ = mcu::Timer::millis();

            if((uint32_t)(mcu::Timer::millis() - user_CHGOCD_ReleaseTimestamp_) > 10UL * 1000UL)
            {
                enableCharging(1 << ERROR_USER_CHG_OCD);
                user_CHGOCD_ReleaseTimestamp_ = 0;
                user_CHGOCD_ReleasedNow_ = true;
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
    cout << stream::Flags::PGM << PSTR("\r\nADCGAIN:        ") << adcGain_;
    cout << stream::Flags::PGM << PSTR("\r\nADCOFFSET:      ") << (uint16_t)adcOffset_;
    cout << stream::Flags::PGM << PSTR("\r\nCHG DIS:        ") << chargingDisabled_;
    cout << stream::Flags::PGM << PSTR("\r\nDISCHG DIS:     ") << dischargingDisabled_ << "\r\n";
}



}
