#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "mcu/watchdog.h"
#include "mcu/serial.h"
#include "mcu/pin.h"
#include "stream/uart0stream.h"


#define PIN_LED_SCK MAKEPIN(B, 5, OUT)

// get free memory
uint16_t freemem() {
    extern int16_t __heap_start, *__brkval;
    int16_t v;
    int16_t Free__Ram = (int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval);
    return (uint16_t)abs(Free__Ram);
}



int main() {
    
    stream::UartStream cout(mcu::Serial0::init());
    mcu::Pin    led(PIN_LED_SCK);
    cout << stream::Flags::PGM << PSTR("HELLO BMS!") << EOL;
    
    led = 1;
    
    while (1) {
        cout << stream::Flags::PGM << PSTR("FREE_RAM: ") << freemem() << EOL;
        _delay_ms(500);
        led = !led; // blink
    }
}
