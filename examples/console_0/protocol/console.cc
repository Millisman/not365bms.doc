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

uint8_t gencrc8(uint8_t *data, uint16_t len) {
    uint8_t crc = 0;
    for (uint16_t i = 0; i < len; i++) {
        crc = devices::_crc8_ccitt_update(crc, data[i]);
    }
    return crc;
}
    
devices::bq769_conf EEMEM In_EEPROM_conf;
    
void Console::conf_load() {
    cout << stream::Flags::PGM << PSTR("conf load ");
    eeprom_read_block(&bq76940_conf, &In_EEPROM_conf, sizeof(bq76940_conf));
    if (bq76940_conf.crc8 != gencrc8((uint8_t*)&bq76940_conf, sizeof(bq76940_conf)-1)) {
        conf_default();
        cout << stream::Flags::PGM << PSTR("crc FAIL, restore defs\r\n");
    } else cout << stream::Flags::PGM << PSTR("OK\r\n");
}

void Console::conf_default() {
    bq76940_conf.m_Debug                    = false;
    bq76940_conf.thermistors_               = MAX_NUMBER_OF_THERMISTORS;
    bq76940_conf.balanceCharging_           = true; // false
    bq76940_conf.autoBalancingEnabled_      = true; // false
    bq76940_conf.shuntResistorValue_uOhm_   = 1000; // 1mOhm
    
    bq76940_conf.thermistorBetaValue_[0]    = 3435;
#ifdef IC_BQ76930
    bq76940_conf.thermistorBetaValue_[1]    = 3435;
#endif
#ifdef IC_BQ76940
    bq76940_conf.thermistorBetaValue_[1]    = 3435;
    bq76940_conf.thermistorBetaValue_[2]    = 3435;
#endif
    
    bq76940_conf.nominalVoltage_            = 3600;     // mV, nominal voltage of single cell in battery pack
    bq76940_conf.fullVoltage_               = 4180;     // mV, full voltage of single cell in battery pack
    bq76940_conf.nominalCapacity_           = 360000;   // mAs (*3600), nominal capacity of battery pack, max. 580 Ah possible @ 3.7V
    bq76940_conf.maxChargeCurrent_          = 5500;     // Current limits (mA)
    bq76940_conf.maxChargeCurrent_delay_    = 3000;     // Current limits Ms
    // setOvercurrentDischargeProtection TODO
    bq76940_conf.idleCurrentThreshold_      = 500;  // 30 Current (mA)
    
    // TODO for any sensors
    bq76940_conf.minCellTempCharge_         =    0; // Temperature limits (C/10)
    bq76940_conf.maxCellTempCharge_         =  500; // Temperature limits (C/10)
    bq76940_conf.minCellTempDischarge_      = -200; // Temperature limits (C/10)
    bq76940_conf.maxCellTempDischarge_      =  650; // Temperature limits (C/10)
    
    bq76940_conf.balancingMinCellVoltage_mV_        = 3600; // Cell voltage (mV)
    bq76940_conf.balancingMaxVoltageDifference_mV_  = 10;   // 20 
    bq76940_conf.balancingMinIdleTime_s_            = 1800;
    
    bq76940_conf.adcPackOffset_             = 0; // mV
    memset(bq76940_conf.adcCellsOffset_, 0, sizeof(bq76940_conf.adcCellsOffset_));
    // Cell voltage limits (mV)
    bq76940_conf.maxCellVoltage_        = 4200; // setting and load
    bq76940_conf.maxCellVoltage_delay_  = 2;    // s
    bq76940_conf.minCellVoltage_        = 2850; // setting and load
    bq76940_conf.minCellVoltage_delay_  = 2;    // s
}

void Console::conf_save() {
    bq76940_conf.ts = mcu::Timer::millis(); // WTF!
    bq76940_conf.crc8 = gencrc8((uint8_t*)&bq76940_conf, sizeof(bq76940_conf)-1);
    eeprom_write_block(&bq76940_conf, &In_EEPROM_conf, sizeof(bq76940_conf));
    cout << stream::Flags::PGM << PSTR("Saved\r\n");
}
    
void Console::write_help(stream::OutputStream &out, const char *cmd, const char *help) {
    out << "  " << stream::PGM << cmd;
    uint8_t len = strlen_P(cmd);
    while (len++ < 20) out << ' ';
    out << stream::PGM << help << EOL;
}
    

Console::Console():
    ser(mcu::Usart::get()),
    cout(ser),
    bq(bq76940_conf, bq76940_data),
//     echo(false),
    handle_result(false),
//     m_interruptFlag(false),
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
}

void Console::begin() {
    bq.begin();
//     bq76940_conf.m_Debug = true;
    postconf_fix();
    bq.update();
    bq.resetSOC(100);
    bq.enableCharging();
    bq.enableDischarging(); // todo
    bq.printRegisters();
    debug_print();
}

// void Console::alertISR() {
//     bq76940_data.alertInterruptFlag_ = true;
// //     m_interruptFlag = true;
// }

bool Console::update(mcu::Pin job, const bool force) {
    bool result = force;
    bq76940_data.alertInterruptFlag_ = force;
    uint32_t now = mcu::Timer::millis();
    if(force || (uint32_t)(now - m_lastUpdate) >= 500) { // 500
        job = 1;
//         if(m_interruptFlag) m_interruptFlag = false;
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
                
        if(bq.batCycles_) {
//             packet.num_cycles += bq.batCycles_;
//             bq.batCycles_ = 0;
//             m_Settings.num_cycles = packet.num_cycles;
//             saveSettings();
        }
                
        if(bq.chargedTimes_) {
//             packet.num_charged += bq.chargedTimes_;
//             m_Settings.num_charged = packet.num_charged;
//             bq.chargedTimes_ = 0;
        }
                
        uint8_t numCells = bq.getNumberOfConnectedCells();
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
    bq.setCellUndervoltageProtection(bq76940_conf.minCellVoltage_, bq76940_conf.minCellVoltage_delay_);
    bq.setCellOvervoltageProtection(bq76940_conf.maxCellVoltage_, bq76940_conf.maxCellVoltage_delay_);
}

void Console::command_apply()   { conf_load(); postconf_fix(); }
void Console::command_restore() { conf_default(); postconf_fix(); }
void Console::command_save()    { conf_save(); }
void Console::command_print() { debug_print(); }
void Console::command_bqregs() { bq.printRegisters(); }
void Console::command_bqdbg() { if (param_len) { bq76940_conf.m_Debug = (bool)atoi(param); } }
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

char const STR_CMD_BQDBG[]      PROGMEM   = "bqdbg";
char const STR_CMD_BQDBG_HLP[]  PROGMEM   = "enable (1) or disable (0) debug events on BQ769x0";
char const STR_CMD_BQREGS[]     PROGMEM   = "bqregs";
char const STR_CMD_BQREGS_HLP[] PROGMEM   = "print regs in BQ769x0";

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
    write_help(cout, STR_CMD_BQDBG,     STR_CMD_BQDBG_HLP);
    write_help(cout, STR_CMD_BQREGS,    STR_CMD_BQREGS_HLP);
    write_help(cout, STR_CMD_PRINT,     STR_CMD_PRINT_HLP);
    write_help(cout, STR_CMD_WDTEST,    STR_CMD_WDTEST_HLP);
    write_help(cout, STR_CMD_BOOTLOADER,STR_CMD_BOOTLOADER_HLP);
    write_help(cout, STR_CMD_FREEMEM,   STR_CMD_FREEMEM_HLP);
    write_help(cout, STR_CMD_EPFORMAT,  STR_CMD_EPFORMAT_HLP);
    write_help(cout, STR_CMD_HELP,      STR_CMD_HELP_HLP);
    write_help(cout, STR_CMD_SHUTDOWN,  STR_CMD_SHUTDOWN_HLP);
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
    compare_cmd(STR_CMD_BQDBG,      &Console::command_bqdbg);
    compare_cmd(STR_CMD_BQREGS,     &Console::command_bqregs);
    compare_cmd(STR_CMD_PRINT,      &Console::command_print);
    compare_cmd(STR_CMD_WDTEST,     &Console::command_wdtest);
    compare_cmd(STR_CMD_BOOTLOADER, &Console::command_bootloader);
    compare_cmd(STR_CMD_FREEMEM,    &Console::command_freemem);
    compare_cmd(STR_CMD_EPFORMAT,   &Console::command_format_EEMEM);
    compare_cmd(STR_CMD_HELP,       &Console::command_help);
    compare_cmd(STR_CMD_SHUTDOWN,   &Console::command_shutdown);
    
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
//             if (len == 0 && ch == '$') echo = false;
//             if (echo) {
                ser.write(ch);
                if (ch == CR) ser.write(LF);
//             }
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
//         echo = true;
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
    cout << stream::Flags::PGM << PSTR("\r\n\r\nXREADY errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_XREADY];
    cout << stream::Flags::PGM << PSTR("\r\n ALERT errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_ALERT];
    cout << stream::Flags::PGM << PSTR("\r\n   UVP errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_UVP];
    cout << stream::Flags::PGM << PSTR("\r\n   OVP errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_OVP];
    cout << stream::Flags::PGM << PSTR("\r\n   SCD errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_SCD];
    cout << stream::Flags::PGM << PSTR("\r\n   OCD errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_OCD];
    cout << stream::Flags::PGM << PSTR("\r\n\r\nDISCHG TEMP errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_USER_DISCHG_TEMP];
    cout << stream::Flags::PGM << PSTR("\r\n   CHG TEMP errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_TEMP];
    cout << stream::Flags::PGM << PSTR("\r\n    CHG OCD errors: ") << bq.errorCounter_[devices::BQ769xERR::ERROR_USER_CHG_OCD] << EOL;
}


}
