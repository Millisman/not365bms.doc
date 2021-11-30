#include <avr/wdt.h>
#include "mcu/watchdog.h"

namespace mcu {    
    void Watchdog::enable() { wdt_enable(WDTO_8S); }
    void Watchdog::reset() { wdt_reset(); }
    void Watchdog::forceRestart() {
        wdt_enable(WDTO_15MS);
        while (1) {
        }
    }

}
