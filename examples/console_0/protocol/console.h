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

#define CONSOLE_BUFFER 64

namespace protocol {

    
enum PrintParam {
    Conf_BQ_dbg,
    Conf_RT_bits,
    Conf_RS_uOhm,
    Conf_RT_Beta,
    Conf_Cell_CapaNom_mV,
    Conf_Cell_CapaFull_mV,
    Conf_Batt_CapaNom_mAsec,
    Conf_CurrentThresholdIdle_mA,
    Conf_Cell_TempCharge_min,
    Conf_Cell_TempCharge_max,
    Conf_Cell_TempDischarge_min,
    Conf_Cell_TempDischarge_max,
    Conf_BalancingInCharge,
    Conf_BalancingEnable,
    Conf_BalancingCellMin_mV,
    Conf_BalancingCellMaxDifference_mV,
    Conf_BalancingIdleTimeMin_s,
    Conf_Cell_OCD_mA,
    Conf_Cell_OCD_ms,
    Conf_Cell_SCD_mA,
    Conf_Cell_SCD_us,
    Conf_Cell_ODP_mA,
    Conf_Cell_ODP_ms,
    Conf_Cell_OVP_mV,
    Conf_Cell_OVP_sec,
    Conf_Cell_UVP_mV,
    Conf_Cell_UVP_sec,
    Conf_adcCellsOffset,
    Conf_ts,
    Conf_CRC8,
    FIRST = Conf_BQ_dbg,
    LAST = Conf_CRC8
};
    
    
    
    
    
    
uint8_t gencrc8(uint8_t *data, uint16_t len);

class Console;
typedef void (Console::*SerialCommandHandler)();
struct SerialCommand { const char *command; SerialCommandHandler handler; };

class Console {
    mcu::Usart &ser;
    stream::UartStream cout;
    devices::bq769_conf bq769x_conf;
    devices::bq769_data bq76940_data;
    devices::bq769x0    bq;
    bool debug_events;
    bool handle_result;
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
    bool Recv();
private:
    void debug_print();
    void conf_load();
    void conf_default();
    void conf_save();

    
    void print_conf(const PrintParam c);
    void print_all_conf();
    
    void command_apply();
    void command_restore();
    void command_save();
    void command_print();
    void command_bqregs();
    void command_wdtest();
    void command_bootloader();
    void command_freemem();
    void command_format_EEMEM();
    void command_help();
    void command_shutdown();
    

    void cmd_BQ_dbg();
    void cmd_RT_bits();
    void cmd_RS_uOhm();
    void cmd_RT_Beta();
    void cmd_Cell_CapaNom_mV();
    void cmd_Cell_CapaFull_mV();
    void cmd_Batt_CapaNom_mAsec();
    void cmd_CurrentThresholdIdle_mA();
    void cmd_Cell_TempCharge_min();
    void cmd_Cell_TempCharge_max();
    void cmd_Cell_TempDischarge_min();
    void cmd_Cell_TempDischarge_max();
    void cmd_BalancingInCharge();
    void cmd_BalancingEnable();
    void cmd_BalancingCellMin_mV();
    void cmd_BalancingCellMaxDifference_mV();
    void cmd_BalancingIdleTimeMin_s();
    void cmd_Cell_OCD_mA();
    void cmd_Cell_OCD_ms();
    void cmd_Cell_SCD_mA();
    void cmd_Cell_SCD_us();
    void cmd_Cell_ODP_mA();  
    void cmd_Cell_ODP_ms();
    void cmd_Cell_OVP_mV();
    void cmd_Cell_OVP_sec();
    void cmd_Cell_UVP_mV();
    void cmd_Cell_UVP_sec();
    
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

