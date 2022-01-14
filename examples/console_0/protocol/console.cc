#include "console.h"
#include <stdlib.h>
#include "mcu/watchdog.h"
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>


namespace {
extern "C" uint16_t get_free_mem() {
    extern int16_t __heap_start, *__brkval;
    int16_t v;
    int16_t Free__Ram = (int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval);
    return (uint16_t)abs(Free__Ram);
}
} // !namespace

#define BackSpace  0x08
#define Delete 0x7F

namespace protocol {

Console::Console():
    ser(mcu::Usart::get()),
    cout(ser),
    bq(bq769x_conf, bq76940_data),
    handle_result(false),
    param_len(0),
    handle_len(0),
    m_lastUpdate(0),
    m_oldMillis(0),
    m_millisOverflows(0),
    len(0),
    state(CONSOLE_STARTUP)
{
    cout << stream::Flags::PGM << PSTR("not365 Console ");
    conf_load();
    
    bq.setShortCircuitProtection(           bq769x_conf.Cell_SCD_mA, bq769x_conf.Cell_SCD_us);
    bq.setOvercurrentChargeProtection(      bq769x_conf.Cell_OCD_mA, bq769x_conf.Cell_OCD_ms);
    bq.setOvercurrentDischargeProtection(   bq769x_conf.Cell_ODP_mA, bq769x_conf.Cell_ODP_ms);
    bq.setCellUndervoltageProtection(       bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec);
    bq.setCellOvervoltageProtection(        bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec);
    
    print_all_conf();
}

void Console::begin() {
    bq.begin();
    postconf_fix();
    bq.update();
    bq.resetSOC(100);
    bq.enableCharging();
    bq.enableDischarging(); // todo
    bq.printRegisters();
    debug_print();
}
    
void Console::conf_default() {
    bq769x_conf.BQ_dbg              = false;
    bq769x_conf.chargingEnabled_    = true;
    bq769x_conf.dischargingEnabled_ = true;
    bq769x_conf.RT_bits    = BQ769X0_THERMISTORS;
    bq769x_conf.RS_uOhm    = 1000; // Shunt, 1mOhm
    bq769x_conf.RT_Beta[0] = 3435;
#ifdef IC_BQ76930
    bq76940_conf.RT_Beta[1] = 3435;
#endif
#ifdef IC_BQ76940
    bq769x_conf.RT_Beta[1] = 3435;
    bq769x_conf.RT_Beta[2] = 3435;
#endif
    // Capacity calc
    bq769x_conf.Cell_CapaNom_mV    = 3600;     // mV, nominal voltage of single cell in battery pack
    bq769x_conf.Cell_CapaFull_mV   = 4180;     // mV, full voltage of single cell in battery pack
    bq769x_conf.Batt_CapaNom_mAsec = 360000;   // mAs (*3600), nominal capacity of battery pack, max. 580 Ah possible @ 3.7V

    bq769x_conf.CurrentThresholdIdle_mA = 100;  // 30 Current (mA)

    // TODO Temperature for any sensors
    bq769x_conf.Cell_TempCharge_min     =    0; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempCharge_max     =  500; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempDischarge_min  = -200; // Temperature limits (Cx10)
    bq769x_conf.Cell_TempDischarge_max  =  650; // Temperature limits (Cx10)
    
    
    bq769x_conf.BalancingInCharge       = true; // false
    bq769x_conf.BalancingEnable         = true; // false
    bq769x_conf.BalancingCellMin_mV     = 3600; // Cell voltage (mV)
    bq769x_conf.BalancingCellMaxDifference_mV   = 10;   // 20 
    bq769x_conf.BalancingIdleTimeMin_s          = 1800;
   
    // checkUser Cell overcurrent charge protection
    bq769x_conf.Cell_OCD_mA    = 5500;     // Current limits (mA)
    bq769x_conf.Cell_OCD_ms    = 3000;     // Current limits Ms
    
    // PROTECT1 Cell short circuit protection
    bq769x_conf.Cell_SCD_mA    = 80000;    // Current limits (mA)
    bq769x_conf.Cell_SCD_us    = 200;      // Current limits us
    
    // PROTECT2 Cell overcurrent discharge protection
    bq769x_conf.Cell_ODP_mA    = 40000;    // Current limits (mA)
    bq769x_conf.Cell_ODP_ms    = 2000;     // Current limits Ms

    // PROTECT3 Cell voltage protection limits
    bq769x_conf.Cell_OVP_mV    = 4200;     // setting and load
    bq769x_conf.Cell_OVP_sec   = 2;        // s
    // min
    bq769x_conf.Cell_UVP_mV    = 2850;     // setting and load
    bq769x_conf.Cell_UVP_sec   = 2;        // s

//     bq769x_conf.adcPackOffset_             = 0; // mV
    memset(bq769x_conf.adcCellsOffset_, 0, sizeof(bq769x_conf.adcCellsOffset_));
    
}




char const STR_cmd_BQ_dbg[]       PROGMEM = "bqdbg";
char const STR_cmd_BQ_dbg_HELP[]  PROGMEM = "on (1) or off (0) debug events on BQ769x0";

void Console::cmd_BQ_dbg() {
    if (param_len) {
        bq769x_conf.BQ_dbg = (bool)atoi(param);    
    }// else write_help(cout, STR_CMD_BQ_DEBUG, STR_CMD_BQ_DEBUG_HELP);
    print_conf(PrintParam::Conf_BQ_dbg);
    
}


char const STR_cmd_RT_bits[]        PROGMEM = "thermistors";
char const STR_cmd_RT_bits_HELP[]   PROGMEM = "<1> <1> <1> - enable 3 of 3";

void Console::cmd_RT_bits() {
    if (param_len) {
        if (param_len == 5) {
            uint8_t a,b,c;
            a = atoi(param);
            b = atoi(param+2);
            c = atoi(param+4);
            if (a) bq769x_conf.RT_bits |= (1 << 0); else bq769x_conf.RT_bits &= ~(1 << 0);
            if (b) bq769x_conf.RT_bits |= (1 << 1); else bq769x_conf.RT_bits &= ~(1 << 1);
            if (c) bq769x_conf.RT_bits |= (1 << 2); else bq769x_conf.RT_bits &= ~(1 << 2);
        } else write_help(cout, STR_cmd_RT_bits, STR_cmd_RT_bits_HELP);
    }
    print_conf(PrintParam::Conf_RT_bits);
}


char const STR_cmd_RS_uOhm[]       PROGMEM = "shuntresistor";
char const STR_cmd_RS_uOhm_HELP[]  PROGMEM = "(1000) = 1mOhm";

void Console::cmd_RS_uOhm() {
    if (param_len) {
        uint32_t sr = 0;
        sr = atoi(param);
        if (sr) bq769x_conf.RS_uOhm = sr;
        else write_help(cout, STR_cmd_RS_uOhm, STR_cmd_RS_uOhm_HELP);
    }
    print_conf(PrintParam::Conf_RS_uOhm);
}


char const STR_cmd_RT_Beta[]       PROGMEM = "thermistorbeta";
char const STR_cmd_RT_Beta_HELP[]  PROGMEM = "(3435) (3435) (3435)";

void Console::cmd_RT_Beta() {
    if (param_len) {
        if (param_len == 14) {
            bq769x_conf.RT_Beta[0] = atoi(param);
            bq769x_conf.RT_Beta[1] = atoi(param+5);
            bq769x_conf.RT_Beta[2] = atoi(param+10);
        } else write_help(cout, STR_cmd_RT_Beta, STR_cmd_RT_Beta_HELP);
    }
    print_conf(PrintParam::Conf_RT_Beta);
}

char const STR_cmd_Cell_CapaNom_mV[]       PROGMEM = "cellnominalmv";
char const STR_cmd_Cell_CapaNom_mV_HELP[]  PROGMEM = "(3600)";

void Console::cmd_Cell_CapaNom_mV() {
    if (param_len) {
        if (param_len == 4) {
            bq769x_conf.Cell_CapaNom_mV = atoi(param);
        } else write_help(cout, STR_cmd_Cell_CapaNom_mV, STR_cmd_Cell_CapaNom_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_CapaNom_mV);
}

char const STR_cmd_Cell_CapaFull_mV[]       PROGMEM = "cellfullmv";
char const STR_cmd_Cell_CapaFull_mV_HELP[]  PROGMEM = "(4200)";

void Console::cmd_Cell_CapaFull_mV() {
    if (param_len) {
        if (param_len == 4) {
            bq769x_conf.Cell_CapaFull_mV = atoi(param);
        } else write_help(cout, STR_cmd_Cell_CapaFull_mV, STR_cmd_Cell_CapaFull_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_CapaFull_mV);
}

char const STR_cmd_Batt_CapaNom_mAsec[]       PROGMEM = "nominalcapacity";
char const STR_cmd_Batt_CapaNom_mAsec_HELP[]  PROGMEM = "ma*h, capacity of battery pack, max. 580 Ah";

void Console::cmd_Batt_CapaNom_mAsec() {
    if (param_len) {
        int32_t t = atol(param);
        if (t) bq769x_conf.Batt_CapaNom_mAsec = t * 60 * 60;
        else write_help(cout, STR_cmd_Batt_CapaNom_mAsec, STR_cmd_Batt_CapaNom_mAsec_HELP);
    }
    print_conf(PrintParam::Conf_Batt_CapaNom_mAsec);
}

char const STR_cmd_CurrentThresholdIdle_mA[]       PROGMEM = "idlecurrentth";
char const STR_cmd_CurrentThresholdIdle_mA_HELP[]  PROGMEM = "mA, for marking 'IDLE', 30-500";

void Console::cmd_CurrentThresholdIdle_mA() {
    if (param_len) {
        uint32_t t = atoi(param);
        if (t > 0) bq769x_conf.CurrentThresholdIdle_mA = t;
        else write_help(cout, STR_cmd_CurrentThresholdIdle_mA, STR_cmd_CurrentThresholdIdle_mA_HELP);
    }
    print_conf(PrintParam::Conf_CurrentThresholdIdle_mA);
}



char const STR_cmd_Cell_TempCharge_min[]       PROGMEM = "celltempchargemin";
char const STR_cmd_Cell_TempCharge_min_HELP[]  PROGMEM = "(0), x10 multipled, less celltempchargemax";

void Console::cmd_Cell_TempCharge_min() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t < bq769x_conf.Cell_TempCharge_max) bq769x_conf.Cell_TempCharge_min = t;
        else write_help(cout, STR_cmd_Cell_TempCharge_min, STR_cmd_Cell_TempCharge_min_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempCharge_min);
}


char const STR_cmd_Cell_TempCharge_max[]       PROGMEM = "celltempchargemax";
char const STR_cmd_Cell_TempCharge_max_HELP[]  PROGMEM = "(0), x10 multipled, above celltempchargemin";

void Console::cmd_Cell_TempCharge_max() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t > bq769x_conf.Cell_TempCharge_min) bq769x_conf.Cell_TempCharge_max = t;
        else write_help(cout, STR_cmd_Cell_TempCharge_max, STR_cmd_Cell_TempCharge_max_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempCharge_max);
}

char const STR_cmd_Cell_TempDischarge_min[]       PROGMEM = "celltempdischargemin";
char const STR_cmd_Cell_TempDischarge_min_HELP[]  PROGMEM = "(0), x10 multipled, less celltempchargemax";

void Console::cmd_Cell_TempDischarge_min() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t < bq769x_conf.Cell_TempDischarge_max) bq769x_conf.Cell_TempDischarge_min = t;
        else write_help(cout, STR_cmd_Cell_TempDischarge_min, STR_cmd_Cell_TempDischarge_min_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempDischarge_min);
}

char const STR_cmd_Cell_TempDischarge_max[]       PROGMEM = "celltempdischargemax";
char const STR_cmd_Cell_TempDischarge_max_HELP[]  PROGMEM = "(0), x10 multipled, above celltempchargemin";

void Console::cmd_Cell_TempDischarge_max() {
    if (param_len) {
        int16_t t = atoi(param);
        if (t > bq769x_conf.Cell_TempDischarge_min) bq769x_conf.Cell_TempDischarge_max = t;
        else write_help(cout, STR_cmd_Cell_TempDischarge_max, STR_cmd_Cell_TempDischarge_max_HELP);
    }
    print_conf(PrintParam::Conf_Cell_TempDischarge_max);
}



char const STR_cmd_BalancingInCharge[]       PROGMEM = "balancecharging";
char const STR_cmd_BalancingInCharge_HELP[]  PROGMEM = "on (1) or off (0) on charging";

void Console::cmd_BalancingInCharge() {
    if (param_len) {
        bq769x_conf.BalancingInCharge = (bool)atoi(param);
    }
    print_conf(PrintParam::Conf_BalancingInCharge);
}

char const STR_cmd_BalancingEnable[]       PROGMEM = "autobalancing";
char const STR_cmd_BalancingEnable_HELP[]  PROGMEM = "on (1) or off (0)";

void Console::cmd_BalancingEnable() {
    if (param_len) {
        bq769x_conf.BalancingEnable = (bool)atoi(param);
    }
    print_conf(PrintParam::Conf_BalancingEnable);
}

char const STR_cmd_BalancingCellMin_mV[]       PROGMEM = "balancingminmv";
char const STR_cmd_BalancingCellMin_mV_HELP[]  PROGMEM = "mV, min for balancing (3600)";

void Console::cmd_BalancingCellMin_mV() {
    if (param_len) {
        int32_t t = atoi(param);
        if (t > 0) bq769x_conf.BalancingCellMin_mV = t;
        else write_help(cout, STR_cmd_BalancingCellMin_mV, STR_cmd_BalancingCellMin_mV_HELP);
    }
    print_conf(PrintParam::Conf_BalancingCellMin_mV);
}


char const STR_cmd_BalancingCellMaxDifference_mV[]       PROGMEM = "balancingmaxdiff";
char const STR_cmd_BalancingCellMaxDifference_mV_HELP[]  PROGMEM = "mV, max for balancing (10)";

void Console::cmd_BalancingCellMaxDifference_mV() {
    if (param_len) {
        uint8_t t = atoi(param);
        if (t > 0) bq769x_conf.BalancingCellMaxDifference_mV = t;
        else write_help(cout, STR_cmd_BalancingCellMaxDifference_mV, STR_cmd_BalancingCellMaxDifference_mV_HELP);
    }
    print_conf(PrintParam::Conf_BalancingCellMaxDifference_mV);
}


char const STR_cmd_BalancingIdleTimeMin_s[]       PROGMEM = "balancingidletime";
char const STR_cmd_BalancingIdleTimeMin_s_HELP[]  PROGMEM = "sec, min value (1800)";

void Console::cmd_BalancingIdleTimeMin_s() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) bq769x_conf.BalancingIdleTimeMin_s = t;
        else write_help(cout, STR_cmd_BalancingIdleTimeMin_s, STR_cmd_BalancingIdleTimeMin_s_HELP);
    }
    print_conf(PrintParam::Conf_BalancingIdleTimeMin_s);
}


char const STR_cmd_Cell_OCD_mA[]       PROGMEM = "maxchargecurrent";
char const STR_cmd_Cell_OCD_mA_HELP[]  PROGMEM = "ma, max charge (5000)";

void Console::cmd_Cell_OCD_mA() {
    if (param_len) {
        int32_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_OCD_mA = t;
            cout << bq.setOvercurrentChargeProtection(bq769x_conf.Cell_OCD_mA, bq769x_conf.Cell_OCD_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_OCD_mA, STR_cmd_Cell_OCD_mA_HELP);
    }
    print_conf(PrintParam::Conf_Cell_OCD_mA);
}

char const STR_cmd_Cell_OCD_ms[]       PROGMEM = "maxchargecurrentdelay";
char const STR_cmd_Cell_OCD_ms_HELP[]  PROGMEM = "ms, overcurrent protect delay (3000)";

void Console::cmd_Cell_OCD_ms() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_OCD_ms = t;
            cout << bq.setOvercurrentChargeProtection(bq769x_conf.Cell_OCD_mA, bq769x_conf.Cell_OCD_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_OCD_ms, STR_cmd_Cell_OCD_ms_HELP);
    }
    print_conf(PrintParam::Conf_Cell_OCD_ms);
}

char const STR_cmd_Cell_SCD_mA[]          PROGMEM = "shortcircuitma";
char const STR_cmd_Cell_SCD_mA_HELP[]     PROGMEM = "ma, trigger current (80000)";

void Console::cmd_Cell_SCD_mA() {
    if (param_len) {
        uint32_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_SCD_mA = t;
            cout << bq.setShortCircuitProtection(bq769x_conf.Cell_SCD_mA, bq769x_conf.Cell_SCD_us) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_SCD_mA, STR_cmd_Cell_SCD_mA_HELP);
    }
    print_conf(PrintParam::Conf_Cell_SCD_mA);
}

char const STR_cmd_Cell_SCD_us[]          PROGMEM = "shortcircuitus";
char const STR_cmd_Cell_SCD_us_HELP[]     PROGMEM = "us, trigger window (200)";

void Console::cmd_Cell_SCD_us() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_SCD_us = t;
            cout << bq.setShortCircuitProtection(bq769x_conf.Cell_SCD_mA, bq769x_conf.Cell_SCD_us) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_SCD_us, STR_cmd_Cell_SCD_us_HELP);
    }
    print_conf(PrintParam::Conf_Cell_SCD_us);
}

char const STR_cmd_Cell_ODP_mA[]          PROGMEM = "dischargema";
char const STR_cmd_Cell_ODP_mA_HELP[]     PROGMEM = "ma, max discharge (40000)";

void Console::cmd_Cell_ODP_mA() {
    if (param_len) {
        uint32_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_ODP_mA = t;
            cout << bq.setOvercurrentDischargeProtection(bq769x_conf.Cell_ODP_mA, bq769x_conf.Cell_ODP_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_ODP_mA, STR_cmd_Cell_ODP_mA_HELP);
    }
    print_conf(PrintParam::Conf_Cell_ODP_mA);
}

char const STR_cmd_Cell_ODP_ms[]          PROGMEM = "dischargems";
char const STR_cmd_Cell_ODP_ms_HELP[]     PROGMEM = "ms, trigger window (2000)";

void Console::cmd_Cell_ODP_ms() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > 0) {
            bq769x_conf.Cell_ODP_ms = t;
            cout << bq.setOvercurrentDischargeProtection(bq769x_conf.Cell_ODP_mA, bq769x_conf.Cell_ODP_ms) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_ODP_ms, STR_cmd_Cell_ODP_ms_HELP);
    }
    print_conf(PrintParam::Conf_Cell_ODP_ms);
}

char const STR_cmd_Cell_OVP_mV[]          PROGMEM = "overvoltagemv";
char const STR_cmd_Cell_OVP_mV_HELP[]     PROGMEM = "mv, limit for cell (4200)";

void Console::cmd_Cell_OVP_mV() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t > bq769x_conf.Cell_UVP_mV) {
            bq769x_conf.Cell_OVP_mV = t;
            cout << bq.setCellOvervoltageProtection(bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_OVP_mV, STR_cmd_Cell_OVP_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_OVP_mV);
}

char const STR_cmd_Cell_OVP_sec[]          PROGMEM = "overvoltagesec";
char const STR_cmd_Cell_OVP_sec_HELP[]     PROGMEM = "sec, trigger window (2)";

void Console::cmd_Cell_OVP_sec() {
    if (param_len) {
        bq769x_conf.Cell_OVP_sec = atoi(param);
        cout << bq.setCellOvervoltageProtection(bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec) << EOL;
    }
    print_conf(PrintParam::Conf_Cell_OVP_sec);
    
}

char const STR_cmd_Cell_UVP_mV[]          PROGMEM = "undervoltagemv";
char const STR_cmd_Cell_UVP_mV_HELP[]     PROGMEM = "mv, limit for cell (2850)";

void Console::cmd_Cell_UVP_mV() {
    if (param_len) {
        uint16_t t = atoi(param);
        if (t < bq769x_conf.Cell_OVP_mV) {
            bq769x_conf.Cell_UVP_mV = t;
            cout << bq.setCellUndervoltageProtection(bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec) << EOL;
        }
        else write_help(cout, STR_cmd_Cell_UVP_mV, STR_cmd_Cell_UVP_mV_HELP);
    }
    print_conf(PrintParam::Conf_Cell_UVP_mV);
}

char const STR_cmd_Cell_UVP_sec[]          PROGMEM = "undervoltagesec";
char const STR_cmd_Cell_UVP_sec_HELP[]     PROGMEM = "sec, trigger window (2)";

void Console::cmd_Cell_UVP_sec() {
    if (param_len) {
        bq769x_conf.Cell_UVP_sec = atoi(param);
        cout << bq.setCellUndervoltageProtection(bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec) << EOL;
    }
    print_conf(PrintParam::Conf_Cell_UVP_sec);
}

// TODO adcCellsOffset_



void Console::print_conf(const PrintParam c) {
    switch (c) {
        case Conf_BQ_dbg:
            cout << stream::Flags::PGM << STR_cmd_BQ_dbg << ':';
            cout << bq769x_conf.BQ_dbg;
            cout << stream::Flags::PGM << STR_cmd_BQ_dbg_HELP;
            break;
        case Conf_RT_bits:
            cout << stream::Flags::PGM << STR_cmd_RT_bits << ':';
            if (bq769x_conf.RT_bits & (1 << 0)) cout << '1'; else cout << '0'; cout << ' ';
            if (bq769x_conf.RT_bits & (1 << 1)) cout << '1'; else cout << '0'; cout << ' ';
            if (bq769x_conf.RT_bits & (1 << 2)) cout << '1'; else cout << '0';
            cout << stream::Flags::PGM << STR_cmd_RT_bits_HELP;
            break;
            
        case Conf_RS_uOhm:
            cout << stream::Flags::PGM << STR_cmd_RS_uOhm << ':';
            cout << bq769x_conf.RS_uOhm;
            cout << stream::Flags::PGM << STR_cmd_RS_uOhm_HELP;
            break;
            
        case Conf_RT_Beta:
            cout << stream::Flags::PGM << STR_cmd_RT_Beta << ':';
            cout << bq769x_conf.RT_Beta[0] << ' ' <<
            bq769x_conf.RT_Beta[1] << ' ' <<
            bq769x_conf.RT_Beta[2];
            cout << stream::Flags::PGM << STR_cmd_RT_Beta_HELP;
            break;
            
        case Conf_Cell_CapaNom_mV:
            cout << stream::Flags::PGM << STR_cmd_Cell_CapaNom_mV << ':';
            cout << bq769x_conf.Cell_CapaNom_mV;
            cout << stream::Flags::PGM << STR_cmd_Cell_CapaNom_mV_HELP;
            break;
            
        case Conf_Cell_CapaFull_mV:
            cout << stream::Flags::PGM << STR_cmd_Cell_CapaFull_mV << ':';
            cout << bq769x_conf.Cell_CapaFull_mV;
            cout << stream::Flags::PGM << STR_cmd_Cell_CapaFull_mV_HELP;
            break;
            
        case Conf_Batt_CapaNom_mAsec:
            cout << stream::Flags::PGM << STR_cmd_Batt_CapaNom_mAsec << ':';
            cout << bq769x_conf.Batt_CapaNom_mAsec/ 60 * 60;
            cout << stream::Flags::PGM << STR_cmd_Batt_CapaNom_mAsec_HELP;
            break;
            
        case Conf_CurrentThresholdIdle_mA:
            cout << stream::Flags::PGM << STR_cmd_CurrentThresholdIdle_mA << ':';
            cout << bq769x_conf.CurrentThresholdIdle_mA;
            cout << stream::Flags::PGM << STR_cmd_CurrentThresholdIdle_mA_HELP;
            break;
            
        case Conf_Cell_TempCharge_min:
            cout << stream::Flags::PGM << STR_cmd_Cell_TempCharge_min << ':';
            cout << bq769x_conf.Cell_TempCharge_min;
            cout << stream::Flags::PGM << STR_cmd_Cell_TempCharge_min_HELP;
            break;
            
        case Conf_Cell_TempCharge_max:
            cout << stream::Flags::PGM << STR_cmd_Cell_TempCharge_max << ':';
            cout << bq769x_conf.Cell_TempCharge_max;
            cout << stream::Flags::PGM << STR_cmd_Cell_TempCharge_max_HELP;
            break;
            
        case Conf_Cell_TempDischarge_min:
            cout << stream::Flags::PGM << STR_cmd_Cell_TempDischarge_min << ':';
            cout << bq769x_conf.Cell_TempDischarge_min;
            cout << stream::Flags::PGM << STR_cmd_Cell_TempDischarge_min_HELP;
            break;
            
        case Conf_Cell_TempDischarge_max:
            cout << stream::Flags::PGM << STR_cmd_Cell_TempDischarge_max << ':';
            cout << bq769x_conf.Cell_TempDischarge_max;
            cout << stream::Flags::PGM << STR_cmd_Cell_TempDischarge_max_HELP;
            break;
            
        case Conf_BalancingInCharge:
            cout << stream::Flags::PGM << STR_cmd_BalancingInCharge << ':';
            cout << bq769x_conf.BalancingInCharge;
            cout << stream::Flags::PGM << STR_cmd_BalancingInCharge_HELP;
            break;
            
        case Conf_BalancingEnable:
            cout << stream::Flags::PGM << STR_cmd_BalancingEnable << ':';
            cout << bq769x_conf.BalancingEnable;
            cout << stream::Flags::PGM << STR_cmd_BalancingEnable_HELP;
            break;
            
        case Conf_BalancingCellMin_mV:
            cout << stream::Flags::PGM << STR_cmd_BalancingCellMin_mV << ':';
            cout << bq769x_conf.BalancingCellMin_mV;
            cout << stream::Flags::PGM << STR_cmd_BalancingCellMin_mV_HELP;
            break;
            
        case Conf_BalancingCellMaxDifference_mV:
            cout << stream::Flags::PGM << STR_cmd_BalancingCellMaxDifference_mV << ':';
            cout << bq769x_conf.BalancingCellMaxDifference_mV;
            cout << stream::Flags::PGM << STR_cmd_BalancingCellMaxDifference_mV_HELP;
            break;
            
        case Conf_BalancingIdleTimeMin_s:
            cout << stream::Flags::PGM << STR_cmd_BalancingIdleTimeMin_s << ':';
            cout << bq769x_conf.BalancingIdleTimeMin_s;
            cout << stream::Flags::PGM << STR_cmd_BalancingIdleTimeMin_s_HELP;
            break;
            
        case Conf_Cell_OCD_mA:
            cout << stream::Flags::PGM << STR_cmd_Cell_OCD_mA << ':';
            cout << bq769x_conf.Cell_OCD_mA;
            cout << stream::Flags::PGM << STR_cmd_Cell_OCD_mA_HELP;
            break;
            
        case Conf_Cell_OCD_ms:
            cout << stream::Flags::PGM << STR_cmd_Cell_OCD_ms << ':';
            cout << bq769x_conf.Cell_OCD_ms;
            cout << stream::Flags::PGM << STR_cmd_Cell_OCD_ms_HELP;
            break;
            
        case Conf_Cell_SCD_mA:
            cout << stream::Flags::PGM << STR_cmd_Cell_SCD_mA << ':';
            cout << bq769x_conf.Cell_SCD_mA;
            cout << stream::Flags::PGM << STR_cmd_Cell_SCD_mA_HELP;
            break;
            
        case Conf_Cell_SCD_us:
            cout << stream::Flags::PGM << STR_cmd_Cell_SCD_us << ':';
            cout << bq769x_conf.Cell_SCD_us;
            cout << stream::Flags::PGM << STR_cmd_Cell_SCD_us_HELP;
            break;
            
        case Conf_Cell_ODP_mA:
            cout << stream::Flags::PGM << STR_cmd_Cell_ODP_mA << ':';
            cout << bq769x_conf.Cell_ODP_mA;
            cout << stream::Flags::PGM << STR_cmd_Cell_ODP_mA_HELP;
            break;
            
        case Conf_Cell_ODP_ms:
            cout << stream::Flags::PGM << STR_cmd_Cell_ODP_ms << ':';
            cout << bq769x_conf.Cell_ODP_ms;
            cout << stream::Flags::PGM << STR_cmd_Cell_ODP_ms_HELP;
            break;
            
        case Conf_Cell_OVP_mV:
            cout << stream::Flags::PGM << STR_cmd_Cell_OVP_mV << ':';
            cout << bq769x_conf.Cell_OVP_mV;
            cout << stream::Flags::PGM << STR_cmd_Cell_OVP_mV_HELP;
            break;
            
        case Conf_Cell_OVP_sec:
            cout << stream::Flags::PGM << STR_cmd_Cell_OVP_sec << ':';
            cout << bq769x_conf.Cell_OVP_sec;
            cout << stream::Flags::PGM << STR_cmd_Cell_OVP_sec_HELP;
            break;
            
        case Conf_Cell_UVP_mV:
            cout << stream::Flags::PGM << STR_cmd_Cell_UVP_mV << ':';
            cout << bq769x_conf.Cell_UVP_mV;
            cout << stream::Flags::PGM << STR_cmd_Cell_UVP_mV_HELP;
            break;
            
        case Conf_Cell_UVP_sec:
            cout << stream::Flags::PGM << STR_cmd_Cell_UVP_sec << ':';
            cout << bq769x_conf.Cell_UVP_sec;
            cout << stream::Flags::PGM << STR_cmd_Cell_UVP_sec_HELP;
            break;
            
        case Conf_adcCellsOffset:
            cout << "TODO";
            break;
            
        case Conf_ts:
            cout << "TS: " << bq769x_conf.ts;
            break;
            
        case Conf_CRC8:
            cout << "CRC8: " << bq769x_conf.crc8;
            break;
        default: {
            cout << '?';
        }
    }
}

void Console::print_all_conf() {
    for (uint8_t i = FIRST ; i <= LAST; i++) {
        print_conf((PrintParam)i);
        cout << EOL;
    }
}


devices::bq769_conf EEMEM In_EEPROM_conf;

void Console::conf_load() {
    cout << stream::Flags::PGM << PSTR("conf load ");
    eeprom_read_block(&bq769x_conf, &In_EEPROM_conf, sizeof(bq769x_conf));
    if (bq769x_conf.crc8 != gencrc8((uint8_t*)&bq769x_conf, sizeof(bq769x_conf)-1)) {
        conf_default();
        cout << stream::Flags::PGM << PSTR("crc FAIL, restore defs\r\n");
        conf_save();
    } else cout << stream::Flags::PGM << PSTR("OK\r\n");
}


void Console::conf_save() {
    bq769x_conf.ts = mcu::Timer::millis(); // WTF!
    bq769x_conf.crc8 = gencrc8((uint8_t*)&bq769x_conf, sizeof(bq769x_conf)-1);
    eeprom_write_block(&bq769x_conf, &In_EEPROM_conf, sizeof(bq769x_conf));
    cout << stream::Flags::PGM << PSTR("Saved\r\n");
}
    
void Console::write_help(stream::OutputStream &out, const char *cmd, const char *help) {
    out << ' ' << stream::PGM << cmd;
    uint8_t len = strlen_P(cmd);
    while (len++ < 24) out << ' ';
    out << ' ' << stream::PGM << help << EOL;
}
    



bool Console::update(mcu::Pin job, const bool force) {
    bool result = force;
    bq76940_data.alertInterruptFlag_ = force;
    uint32_t now = mcu::Timer::millis();
    if(force || (uint32_t)(now - m_lastUpdate) >= 500) { // 500
        job = 1;
        uint8_t error = bq.update(); // should be called at least every 250 ms
        m_lastUpdate = now;

        // charging state
        //if(bq.getBatteryCurrent()       > (int16_t)m_Settings.idle_currentThres) packet.status |= (1 << 6); // charging
        //else if(bq.getBatteryCurrent()  < (int16_t)m_Settings.idle_currentThres / 2) packet.status &= ~(1 << 6);
        if(error & STAT_OV) { cout << stream::Flags::PGM << PSTR("overvoltage\r\n"); }
            //packet.status |= (1 << 9); error &= ~STAT_OV; } // overvoltage
        //else packet.status &= ~(1 << 9);
        
//         uint16_t batVoltage = bq.getBatteryVoltage() / 10;
//         if(batVoltage > packet.max_voltage) packet.max_voltage = batVoltage;
//         int16_t batCurrent = bq.getBatteryCurrent() / 10;
//         if(batCurrent > 0 && (uint16_t)batCurrent > packet.max_charge_current)
//             packet.max_charge_current = batCurrent;
//         else if(batCurrent < 0 && (uint16_t)-batCurrent > packet.max_discharge_current)
//             packet.max_discharge_current = -batCurrent;
//             0
        // packet.capacity_left = packet.design_capacity * bq.getSOC() / 100.0;
        // packet.percent_left = bq.getSOC();
        //packet.current = -batCurrent;
        //packet.voltage = batVoltage;
        //packet.temperature[0] = bq.getTemperatureDegC(1) + 20.0;
        //packet.temperature[1] = bq.getTemperatureDegC(2) + 20.0;
            
        //if(bq.getHighestTemperature() > (m_Settings.temp_maxDischargeC - 3) * 10)
        //    packet.status |= (1 << 10); // overheat
        //else
        //    packet.status &= ~(1 << 10);
                
        if(bq76940_data.batCycles_) {
//             packet.num_cycles += bq76940_data.batCycles_;
//             bq76940_data.batCycles_ = 0;
//             m_Settings.num_cycles = packet.num_cycles;
//             saveSettings();
        }
                
        if(bq76940_data.chargedTimes_) {
//             packet.num_charged += bq76940_data.chargedTimes_;
//             m_Settings.num_charged = packet.num_charged;
//             bq76940_data.chargedTimes_ = 0;
        }
                
//        uint8_t numCells = bq.getNumberOfConnectedCells();
//         for(uint8_t i = 0; i < numCells; i++)
//             packet.cell_voltages[i] = bq.getCellVoltage(i);
        
        // cell voltage difference too big
        uint16_t bigDelta = bq.getMaxCellVoltage() - bq.getMinCellVoltage();
        if(bigDelta > 100) cout << stream::Flags::PGM << PSTR("difference too big\r\n");
//             error = 1;
//         
//         if(error)
//             packet.status &= ~1;
//         else
//             packet.status |= 1;

        if(m_oldMillis > now)
            m_millisOverflows++;
        m_oldMillis = now;
        job = 0;
    }
    
    return result;

//     return Recv();
}

void Console::postconf_fix() {
    bq.setCellUndervoltageProtection(bq769x_conf.Cell_UVP_mV, bq769x_conf.Cell_UVP_sec);
    bq.setCellOvervoltageProtection(bq769x_conf.Cell_OVP_mV, bq769x_conf.Cell_OVP_sec);
}

void Console::command_apply()   { conf_load(); postconf_fix(); }
void Console::command_restore() { conf_default(); postconf_fix(); }
void Console::command_save()    { conf_save(); }
void Console::command_print() { debug_print(); }
void Console::command_bqregs() { bq.printRegisters(); }
// void Console::cmd_bq_debug() { if (param_len) { bq76940_conf.m_Debug = (bool)atoi(param); } }
void Console::command_wdtest() { for (;;) { (void)0; } }
void Console::command_freemem() { cout << stream::Flags::PGM << PSTR(" Free RAM:") << get_free_mem() << EOL; }



char const STR_CMD_APPLY[]          PROGMEM = "apply";
char const STR_CMD_APPLY_HLP[]      PROGMEM = "apply settings";

char const STR_CMD_RESTORE[]        PROGMEM = "restore";
char const STR_CMD_RESTORE_HLP[]    PROGMEM = "restore defaults settings";

char const STR_CMD_SAVE[]           PROGMEM = "save";
char const STR_CMD_SAVE_HLP[]       PROGMEM = "save current settings";

char const STR_CMD_PRINT[]          PROGMEM = "print";
char const STR_CMD_PRINT_HLP[]      PROGMEM = "print status";

char const STR_CMD_WDTEST[]         PROGMEM = "wdtest";
char const STR_CMD_WDTEST_HLP[]     PROGMEM = "test watchdog";

char const STR_CMD_BOOTLOADER[]     PROGMEM = "bootloader";
char const STR_CMD_BOOTLOADER_HLP[] PROGMEM = "jump to bootloader";

char const STR_CMD_FREEMEM[]        PROGMEM = "freemem";
char const STR_CMD_FREEMEM_HLP[]    PROGMEM = "show free memory in heap";

char const STR_CMD_EPFORMAT[]       PROGMEM = "format";
char const STR_CMD_EPFORMAT_HLP[]   PROGMEM = "formating EEPROM (load defaults settings on next boot)";

char const STR_CMD_HELP[]           PROGMEM = "help";
char const STR_CMD_HELP_HLP[]       PROGMEM = "this 'help'";


char const STR_CMD_BQREGS[]         PROGMEM = "bqregs";
char const STR_CMD_BQREGS_HLP[]     PROGMEM = "print regs in BQ769x0";

char const STR_CMD_SHUTDOWN[]       PROGMEM = "shutdown";
char const STR_CMD_SHUTDOWN_HLP[]   PROGMEM = "bye...bye...'";


void Console::command_shutdown() {
    //TODO save data, stats
    cout << stream::PGM << STR_CMD_SHUTDOWN_HLP;
    bq.shutdown();
}








void Console::command_help() {
    cout << stream::PGM << PSTR("Available commands:\r\n") << EOL;
    write_help(cout, STR_CMD_APPLY,     STR_CMD_APPLY_HLP);
    write_help(cout, STR_CMD_RESTORE,   STR_CMD_RESTORE_HLP);
    write_help(cout, STR_CMD_SAVE,      STR_CMD_SAVE_HLP);
    write_help(cout, STR_CMD_BQREGS,    STR_CMD_BQREGS_HLP);
    write_help(cout, STR_CMD_PRINT,     STR_CMD_PRINT_HLP);
    write_help(cout, STR_CMD_WDTEST,    STR_CMD_WDTEST_HLP);
    write_help(cout, STR_CMD_BOOTLOADER,STR_CMD_BOOTLOADER_HLP);
    write_help(cout, STR_CMD_FREEMEM,   STR_CMD_FREEMEM_HLP);
    write_help(cout, STR_CMD_EPFORMAT,  STR_CMD_EPFORMAT_HLP);
    write_help(cout, STR_CMD_HELP,      STR_CMD_HELP_HLP);
    write_help(cout, STR_CMD_SHUTDOWN,  STR_CMD_SHUTDOWN_HLP);
    
    write_help(cout, STR_cmd_BQ_dbg,  STR_cmd_BQ_dbg_HELP); 
    write_help(cout, STR_cmd_RT_bits,  STR_cmd_RT_bits_HELP); 
    write_help(cout, STR_cmd_RS_uOhm,  STR_cmd_RS_uOhm_HELP); 
    write_help(cout, STR_cmd_RT_Beta,  STR_cmd_RT_Beta_HELP); 
    write_help(cout, STR_cmd_Cell_CapaNom_mV,  STR_cmd_Cell_CapaNom_mV_HELP); 
    write_help(cout, STR_cmd_Cell_CapaFull_mV,  STR_cmd_Cell_CapaFull_mV_HELP); 
    write_help(cout, STR_cmd_Batt_CapaNom_mAsec,  STR_cmd_Batt_CapaNom_mAsec_HELP); 
    write_help(cout, STR_cmd_CurrentThresholdIdle_mA,  STR_cmd_CurrentThresholdIdle_mA_HELP); 
    write_help(cout, STR_cmd_Cell_TempCharge_min,  STR_cmd_Cell_TempCharge_min_HELP); 
    write_help(cout, STR_cmd_Cell_TempCharge_max,  STR_cmd_Cell_TempCharge_max_HELP); 
    write_help(cout, STR_cmd_Cell_TempDischarge_min,  STR_cmd_Cell_TempDischarge_min_HELP); 
    write_help(cout, STR_cmd_Cell_TempDischarge_max,  STR_cmd_Cell_TempDischarge_max_HELP); 
    write_help(cout, STR_cmd_BalancingInCharge,  STR_cmd_BalancingInCharge_HELP); 
    write_help(cout, STR_cmd_BalancingEnable,  STR_cmd_BalancingEnable_HELP); 
    write_help(cout, STR_cmd_BalancingCellMin_mV,  STR_cmd_BalancingCellMin_mV_HELP); 
    write_help(cout, STR_cmd_BalancingCellMaxDifference_mV,  STR_cmd_BalancingCellMaxDifference_mV_HELP); 
    write_help(cout, STR_cmd_BalancingIdleTimeMin_s,  STR_cmd_BalancingIdleTimeMin_s_HELP); 
    write_help(cout, STR_cmd_Cell_OCD_mA,  STR_cmd_Cell_OCD_mA_HELP); 
    write_help(cout, STR_cmd_Cell_OCD_ms,  STR_cmd_Cell_OCD_ms_HELP); 
    write_help(cout, STR_cmd_Cell_SCD_mA,  STR_cmd_Cell_SCD_mA_HELP); 
    write_help(cout, STR_cmd_Cell_SCD_us,  STR_cmd_Cell_SCD_us_HELP); 
    write_help(cout, STR_cmd_Cell_ODP_mA,  STR_cmd_Cell_ODP_mA_HELP); 
    write_help(cout, STR_cmd_Cell_ODP_ms,  STR_cmd_Cell_ODP_ms_HELP); 
    write_help(cout, STR_cmd_Cell_OVP_mV,  STR_cmd_Cell_OVP_mV_HELP); 
    write_help(cout, STR_cmd_Cell_OVP_sec,  STR_cmd_Cell_OVP_sec_HELP); 
    write_help(cout, STR_cmd_Cell_UVP_mV,  STR_cmd_Cell_UVP_mV_HELP); 
    write_help(cout, STR_cmd_Cell_UVP_sec,  STR_cmd_Cell_UVP_sec_HELP); 
    
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



// char const STR_CMD_SHUTDOWN[]       PROGMEM = "shutdown";
// char const STR_CMD_SHUTDOWN_HLP[]   PROGMEM = "bye...bye...'";
// 


bool Console::handleCommand(const char *buffer, const uint8_t len) {
    if (buffer[0] == 0) return false;
    handle_result = false;
    handle_buffer = buffer;
    handle_len = len;    
    compare_cmd(STR_CMD_APPLY,      &Console::command_apply);
    compare_cmd(STR_CMD_RESTORE,    &Console::command_restore);
    compare_cmd(STR_CMD_SAVE,       &Console::command_save);
    compare_cmd(STR_CMD_BQREGS,     &Console::command_bqregs);
    compare_cmd(STR_CMD_PRINT,      &Console::command_print);
    compare_cmd(STR_CMD_WDTEST,     &Console::command_wdtest);
    compare_cmd(STR_CMD_BOOTLOADER, &Console::command_bootloader);
    compare_cmd(STR_CMD_FREEMEM,    &Console::command_freemem);
    compare_cmd(STR_CMD_EPFORMAT,   &Console::command_format_EEMEM);
    compare_cmd(STR_CMD_HELP,       &Console::command_help);
    compare_cmd(STR_CMD_SHUTDOWN,   &Console::command_shutdown);
    
    
    compare_cmd(STR_cmd_BQ_dbg,                 &Console::cmd_BQ_dbg);
    compare_cmd(STR_cmd_RT_bits,                &Console::cmd_RT_bits);
    compare_cmd(STR_cmd_RS_uOhm,                &Console::cmd_RS_uOhm);
    compare_cmd(STR_cmd_RT_Beta,                &Console::cmd_RT_Beta);
    compare_cmd(STR_cmd_Cell_CapaNom_mV,        &Console::cmd_Cell_CapaNom_mV);
    compare_cmd(STR_cmd_Cell_CapaFull_mV,       &Console::cmd_Cell_CapaFull_mV);
    compare_cmd(STR_cmd_Batt_CapaNom_mAsec,     &Console::cmd_Batt_CapaNom_mAsec);
    compare_cmd(STR_cmd_CurrentThresholdIdle_mA,&Console::cmd_CurrentThresholdIdle_mA);
    compare_cmd(STR_cmd_Cell_TempCharge_min,    &Console::cmd_Cell_TempCharge_min);
    compare_cmd(STR_cmd_Cell_TempCharge_max,    &Console::cmd_Cell_TempCharge_max);
    compare_cmd(STR_cmd_Cell_TempDischarge_min, &Console::cmd_Cell_TempDischarge_min);
    compare_cmd(STR_cmd_Cell_TempDischarge_max, &Console::cmd_Cell_TempDischarge_max);
    compare_cmd(STR_cmd_BalancingInCharge,      &Console::cmd_BalancingInCharge);
    compare_cmd(STR_cmd_BalancingEnable,        &Console::cmd_BalancingEnable);
    compare_cmd(STR_cmd_BalancingCellMin_mV,    &Console::cmd_BalancingCellMin_mV);
    compare_cmd(STR_cmd_BalancingCellMaxDifference_mV, &Console::cmd_BalancingCellMaxDifference_mV);
    compare_cmd(STR_cmd_BalancingIdleTimeMin_s, &Console::cmd_BalancingIdleTimeMin_s);
    compare_cmd(STR_cmd_Cell_OCD_mA,            &Console::cmd_Cell_OCD_mA);
    compare_cmd(STR_cmd_Cell_OCD_ms,            &Console::cmd_Cell_OCD_ms);
    compare_cmd(STR_cmd_Cell_SCD_mA,            &Console::cmd_Cell_SCD_mA);
    compare_cmd(STR_cmd_Cell_SCD_us,            &Console::cmd_Cell_SCD_us);
    compare_cmd(STR_cmd_Cell_ODP_mA,            &Console::cmd_Cell_ODP_mA);  
    compare_cmd(STR_cmd_Cell_ODP_ms,            &Console::cmd_Cell_ODP_ms);
    compare_cmd(STR_cmd_Cell_OVP_mV,            &Console::cmd_Cell_OVP_mV);
    compare_cmd(STR_cmd_Cell_OVP_sec,           &Console::cmd_Cell_OVP_sec);
    compare_cmd(STR_cmd_Cell_UVP_mV,            &Console::cmd_Cell_UVP_mV);
    compare_cmd(STR_cmd_Cell_UVP_sec,           &Console::cmd_Cell_UVP_sec);
    
    
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
            result = true;
            ch = ser.read();
                ser.write(ch);
                if (ch == CR) ser.write(LF);
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
        cout << "\r\nBMS>";
        len = 0;
        state = CONSOLE_ACCUMULATING;
    }
    return result;
}


void Console::debug_print() {
    uint32_t uptime = m_millisOverflows * (0xffffffffLL / 1000UL);
    uptime += mcu::Timer::millis() / 1000;
    cout << stream::Flags::PGM << PSTR("uptime: ") << uptime << EOL;
    cout << stream::Flags::PGM << 
    PSTR("Battery voltage: ") << bq76940_data.batVoltage_ << stream::Flags::PGM <<
    PSTR(" (") << bq76940_data.batVoltage_raw_ << stream::Flags::PGM << PSTR(")\r\n");
    
    cout << stream::Flags::PGM <<
    PSTR("Battery current: ") << bq76940_data.batCurrent_ << stream::Flags::PGM <<
    PSTR(" (") << bq76940_data.batCurrent_raw_ << stream::Flags::PGM << PSTR(")\r\n"); // TODO
    
    cout << stream::Flags::PGM << PSTR("SOC: ") << bq.getSOC() << EOL; 
    
    cout << stream::Flags::PGM << PSTR("Temperature: ") <<
    bq.getTemperatureDegC(0) << ' ' <<
    bq.getTemperatureDegC(1) << ' ' <<
    bq.getTemperatureDegC(2) << EOL;
    uint8_t numCells = bq.getNumberOfCells();
    cout << stream::Flags::PGM << PSTR("Balancing status: ") << bq76940_data.balancingStatus_ << EOL;
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
    cout << stream::Flags::PGM << PSTR("\r\n\r\nXREADY errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_XREADY];
    cout << stream::Flags::PGM << PSTR("\r\n ALERT errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_ALERT];
    cout << stream::Flags::PGM << PSTR("\r\n   UVP errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_UVP];
    cout << stream::Flags::PGM << PSTR("\r\n   OVP errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_OVP];
    cout << stream::Flags::PGM << PSTR("\r\n   SCD errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_SCD];
    cout << stream::Flags::PGM << PSTR("\r\n   OCD errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_OCD];
    cout << stream::Flags::PGM << PSTR("\r\n\r\nDISCHG TEMP errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_USER_DISCHG_TEMP];
    cout << stream::Flags::PGM << PSTR("\r\n   CHG TEMP errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_TEMP];
    cout << stream::Flags::PGM << PSTR("\r\n    CHG OCD errors: ") << bq76940_data.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_OCD] << EOL;
}

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

uint8_t gencrc8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = devices::_crc8_ccitt_update(crc, data[i]);
    }
    return crc;
}

void Console::command_format_EEMEM() {
    for (int i = 0 ; i < E2END + 1 ; i++) {
        eeprom_write_byte((uint8_t*)i, 0xff);
        ser.write('.');
        mcu::Watchdog::reset();
    }
    cout << EOL;
}


}
