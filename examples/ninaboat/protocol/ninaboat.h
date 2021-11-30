#pragma once

#include <stdint.h>
#include "utils/cpp.h"
#include "stream/uartstream.h"
#include <string.h>
#include <avr/eeprom.h>
#include "devices/bq769x0.h"
#include "mcu/timer.h"
#include <avr/pgmspace.h>
#include "mcu/pin.h"

#define M365BMS_RADDR 0x22
#define M365BMS_WADDR 0x25

#define BQ769X0_DEBUG 1


namespace protocol {
/*
struct BMSSettings {
    uint8_t header[2] = {0xB0, 0x0B};
    uint16_t version = 1;
    char serial[14] = "Lanthi 12s5p";//put name BMS you want it will only see 11 characters
    uint32_t capacity = 13800; // mAh Put here your real milliamps if you have an amp tester look at your real battery at the output
    uint16_t nominal_voltage = 3600; // mV
    uint16_t full_voltage = 4175; // mV
    uint16_t num_cycles = 0;
    uint16_t num_charged = 0;
    uint16_t date = (20 << 9) | (11 << 5) | 10; // MSB (7 bits year, 4 bits month, 5 bits day) LSB
    
    // setShuntResistorValue
    uint16_t shuntResistor_uOhm = 1000;
    
    // setThermistorBetaValue
    uint16_t thermistor_BetaK = 3435;
    
    // setTemperatureLimits
    int16_t temp_minDischargeC = -20; // °C
    int16_t temp_maxDischargeC = 60; // °C
    int16_t temp_minChargeC = 0; // °C
    int16_t temp_maxChargeC = 45; // °C
    
    // setShortCircuitProtection
    uint32_t SCD_current = 80000; // mA
    uint16_t SCD_delay = 200; // us
    
    // setOvercurrentChargeProtection
    uint32_t OCD_current = 5000; // mA
    uint16_t OCD_delay = 3000; // ms
    
    // setOvercurrentDischargeProtection
    uint32_t ODP_current = 35000; // mA
    uint16_t ODP_delay = 1280; // ms
    
    // setCellUndervoltageProtection
    uint16_t UVP_voltage = 2800; // mV
    uint16_t UVP_delay = 2; // s
    
    // setCellOvervoltageProtection
    uint16_t OVP_voltage = 4200; // mV
    uint16_t OVP_delay = 2; // s
    
    // setBalancingThresholds
    uint16_t balance_minIdleTime = 1800; // s
    uint16_t balance_minVoltage = 3600; // mV
    uint16_t balance_maxVoltageDiff = 10; // mV
    
    // setIdleCurrentThreshold
    uint16_t idle_currentThres = 500; // mA
    
    // enableAutoBalancing
    uint16_t balance_enabled = 1;
    
    // adjADCPackOffset
    int16_t adcPackOffset = 0;
    
    // adjADCCellsOffset
    int16_t adcCellsOffset[15] = {0};
    
} __attribute__((packed));
*/






struct BMSSettings {
    uint8_t header[2];
    uint16_t version;
    char serial[14];                    //put name BMS you want it will only see 11 characters
    uint32_t capacity;                  // mAh Put here your real milliamps if you have an amp
                                        // tester look at your real battery at the output
    uint16_t nominal_voltage;           // mV
    uint16_t full_voltage;              // mV
    uint16_t num_cycles;
    uint16_t num_charged;
    uint16_t date;                      // MSB (7 bits year, 4 bits month, 5 bits day) LSB
    uint16_t shuntResistor_uOhm;        // setShuntResistorValue
    uint16_t thermistor_BetaK;          // setThermistorBetaValue
    int16_t temp_minDischargeC;         // °C // setTemperatureLimits
    int16_t temp_maxDischargeC;         // °C
    int16_t temp_minChargeC;            // °C
    int16_t temp_maxChargeC;            // °C
    uint32_t SCD_current;               // mA // setShortCircuitProtection
    uint16_t SCD_delay;                 // us
    uint32_t OCD_current;               // mA // setOvercurrentChargeProtection
    uint16_t OCD_delay;                 // ms
    uint32_t ODP_current;               // mA // setOvercurrentDischargeProtection
    uint16_t ODP_delay;                 // ms
    uint16_t UVP_voltage;               // mV // setCellUndervoltageProtection
    uint16_t UVP_delay;                 // s
    uint16_t OVP_voltage;               // mV // setCellOvervoltageProtection
    uint16_t OVP_delay;                 // s
    uint16_t balance_minIdleTime;       // s // setBalancingThresholds
    uint16_t balance_minVoltage;        // mV
    uint16_t balance_maxVoltageDiff;    // mV
    uint16_t idle_currentThres;         // mA // setIdleCurrentThreshold
    uint16_t balance_enabled;           // enableAutoBalancing
    int16_t adcPackOffset;              // adjADCPackOffset
    int16_t adcCellsOffset[15];         // adjADCCellsOffset
} __attribute__((packed));









struct m365pkt { // little endian
    /*00-1F*/   uint16_t unk1[16] = {0x5A, 0x5A, 0x00};
    /*20-2D*/   char serial[14] = "";
    /*2E-2F*/   uint16_t version = 0x900; // 0x115 = 1.1.5
    /*30-31*/   uint16_t design_capacity = 0; // mAh
    /*32-33*/   uint16_t real_capacity = 0; // mAh
    /*34-35*/   uint16_t nominal_voltage = 0; // mV
    /*36-37*/   uint16_t num_cycles = 0;
    /*38-39*/   uint16_t num_charged = 0;
    /*3A-3B*/   uint16_t max_voltage = 0; // V/100
    /*3C-3D*/   uint16_t max_discharge_current = 0; // A/100
    /*3E-3F*/   uint16_t max_charge_current = 0; // A/100
    /*40-41*/   uint16_t date = 0; // MSB (7 bits year, 4 bits month, 5 bits day) LSB
    /*42-47*/   uint8_t errors[6] = {0};
    /*48-5F*/   uint16_t unk3[12] = {0};
    /*60-61*/   uint16_t status = 1; // b0 = config valid, b6 = charging, b9 = overvoltage, b10 = overheat
    /*62-63*/   uint16_t capacity_left = 0; // mAh
    /*64-65*/   uint16_t percent_left = 0;
    /*66-67*/   int16_t current = 0; // A/100
    /*68-69*/   uint16_t voltage = 0; // V/100
    /*6A-6B*/   uint8_t temperature[2] = {0, 0}; // °C - 20
    /*6C-6D*/   uint16_t balance_bits = 0;
    /*6E-75*/   uint16_t unk5[4] = {0};
    /*76-77*/   uint16_t health = 100; // %, <60% = battery bad
    /*78-7F*/   uint16_t unk6[4] = {0};
    /*80-9D*/   uint16_t cell_voltages[15] = {0}; // mV
    /*9E-A1*/   uint16_t unk7[2] = {0};
#if 0
    /*A2-A3*/   uint16_t unk8 = 1; // 1 ?
    /*A4-DF*/   uint16_t unk9[30] = {0};
    /*E0-E0*/   uint8_t unk10 = 0x3F; // BMS specific value ?
    /*E1-E1*/   uint8_t unk11 = 0; // 0 ?
    /*E2-E2*/   uint8_t unk12 = 0x3C; // BMS specific value ?
    /*E3-E4*/   uint16_t unk13 = 1; // 1 ?
    /*E5-EB*/   char unk_serial[7] = "G55179"; // BMS specific value ?
    /*EC-FF*/   uint8_t unk14[19];
#endif
} __attribute__((packed));

struct NinaBoatMessage {
    uint8_t header[2]; // 0x55, 0xAA
    uint8_t length; // length of data + 2
    uint8_t addr; // receiver address
    uint8_t mode; // read = 1 / write = 3
    uint8_t offset; // data offset/index in array
    uint8_t data[253]; // write = data, read = uint8_t read length  
    uint16_t checksum; // (bytes without header) XOR 0xFFFF
};
    
    
    

    
    
    
    
    
    
    
class NinaBoat {
    mcu::Usart &ser;
    stream::UartStream cout;
    m365pkt packet;
    BMSSettings m_Settings;
    devices::bq769x0 bq;
public:
    NinaBoat();
    bool update(mcu::Pin job);
    void print();
    void start();
    void alertISR();
    void onMessage(NinaBoatMessage &msg);
    void Send(NinaBoatMessage &msg);
    bool Recv();
    void debug_print();
    void applySettings();
    void loadSettings();
    void saveSettings();
    void setDebug(bool dbg);
private:
    void loadSettingsDefault();
    bool m_Debug = true;
    bool m_interruptFlag = false;
    uint32_t m_lastUpdate = 0;

    // ---------------------------------------- TO METHOD OVEFLOV
    uint32_t m_oldMillis = 0;
    int m_millisOverflows = 0;
    
    uint32_t timer0_millis;
    uint16_t m_timer2Overflows = 0;
    
    
    
    
    
    NinaBoatMessage msg;
    uint8_t recvd = 0;
    uint32_t begin = 0;
    uint16_t checksum;
    
    
};

}

