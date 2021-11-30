#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "mcu/watchdog.h"
#include "mcu/usart.h"
#include "mcu/pin.h"



#define PIN_LED_SCK MAKEPIN(B, 5, OUT)

int main() {
    sei();
    mcu::Usart &uart = mcu::Usart::get();
    mcu::Pin    led(PIN_LED_SCK);  
    led = 1;
    
    uart.write('+');
    
    while (1) {
        if (uart.avail()) {
            uint8_t ch = uart.read();
            uart.write(ch);
            led = !led; // blink
        }
        
    }
    
//     uint8_t u = 0;
//     while (1) {
//     uart.write(u++);
//     led = !led; // blink
//     }
    
}
