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

namespace protocol {

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
    

#define CONSOLE_BUFFER 64


class Console;
typedef void (Console::*SerialCommandHandler)();

struct SerialCommand { const char *command; SerialCommandHandler handler; };


class Console {
    mcu::Usart &ser;
    stream::UartStream cout;
    BMSSettings m_Settings;
    devices::bq769x0 bq;
    
    
    
    const char *param;
    uint8_t param_len;
    
    bool debug_events;
    void compare_cmd(const char *name_P, SerialCommandHandler handler);
    const char   *handle_buffer;
    uint8_t handle_len;
    bool    handle_result;
    
public:
    Console();
    bool update(mcu::Pin job);
//     void print();
    void start();
    void alertISR();
//     void onMessage(NinaBoatMessage &msg);
//     void Send(NinaBoatMessage &msg);
    bool Recv();
    void debug_print();
    void applySettings();
    void loadSettings();
    void saveSettings();
//     void setDebug(bool dbg);
private:
    void command_apply();
    void command_restore();
    void command_save();
    void command_print();
    void command_bqdbg();
    void command_bqregs();
    void command_wdtest();
    void command_bootloader();
    void command_freemem();
    void command_format_EEMEM();
    
    void commandHelp();
    bool handleCommand(const char *buffer, const uint8_t len);
    void write_help(stream::OutputStream &out, const char *cmd, const char *help);
    
    enum SerialState { CONSOLE_STARTUP, CONSOLE_ACCUMULATING, CONSOLE_COMMAND };
    SerialState state;
    void loadSettingsDefault();
//     bool m_Debug = true;
    bool m_interruptFlag = false;
    uint32_t m_lastUpdate = 0;

    // ---------------------------------------- TO METHOD OVEFLOV
    uint32_t m_oldMillis = 0;
    int m_millisOverflows = 0;    
    uint32_t timer0_millis;
    uint16_t m_timer2Overflows = 0;

//     uint8_t recvd = 0;
//     uint32_t begin = 0;
    uint16_t checksum;
    uint8_t len = 0;
    bool echo;
    char buffer[CONSOLE_BUFFER];
    
};

}

