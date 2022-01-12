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

#define CONSOLE_BUFFER 64

class Console;
typedef void (Console::*SerialCommandHandler)();
struct SerialCommand { const char *command; SerialCommandHandler handler; };

class Console {
    mcu::Usart &ser;
    stream::UartStream cout;
    devices::bq769_conf bq76940_conf;
    devices::bq769_data bq76940_data;
    devices::bq769x0    bq;

//     bool echo;
    bool debug_events;
    bool handle_result;
//     bool m_interruptFlag = false;
    uint8_t param_len;
    uint8_t handle_len;
    uint32_t m_lastUpdate;
    uint32_t m_oldMillis = 0;
    uint32_t m_millisOverflows;
    uint8_t len;

public:
    Console();
    bool update(mcu::Pin job, const bool force);
    void begin();
//     void alertISR();
    bool Recv();
private:
    void debug_print();
    void conf_load();
    void conf_default();
    void conf_save();
    
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
    void command_help();
    void command_shutdown();
    
    
    bool handleCommand(const char *buffer, const uint8_t len);
    void write_help(stream::OutputStream &out, const char *cmd, const char *help);
    char buffer[CONSOLE_BUFFER];
    void postconf_fix();
    void compare_cmd(const char *name_P, SerialCommandHandler handler);
    enum SerialState { CONSOLE_STARTUP, CONSOLE_ACCUMULATING, CONSOLE_COMMAND };
    SerialState state;
    const char *handle_buffer;
    const char *param;
};

}

