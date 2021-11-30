#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <util/delay.h>
#include <stdlib.h>
#include <math.h>
#include "mcu/watchdog.h"
#include "mcu/usart.h"
#include "mcu/pin.h"
#include "stream/uartstream.h"
#include <avr/sleep.h>
#include "protocol/ninaboat.h"

protocol::NinaBoat proto; // device protocol class

uint32_t g_lastActivity = 0;
bool g_Debug = true;

#define PIN_LED_SCK MAKEPIN(B, 5, OUT)

volatile uint16_t g_timer2Overflows = 0;
bool g_wakeupFlag = false;
bool g_uartRxInterrupted = false;


// ISR(INT1_vect)
// ISR(INT2_vect)
ISR(INT0_vect) {
    g_wakeupFlag = true;
    proto.alertISR();
}

ISR(PCINT2_vect) {
    g_wakeupFlag = true;
    g_uartRxInterrupted = true;
}


#define CHANGE 1
#define FALLING 2
#define RISING 3

void activate_INT0() {
    EICRA = (1 << ISC01) | (1 << ISC00) | (RISING << ISC00);
    EIMSK = (1 << INT0);
    //----------------------------------------------------------------- INT1_vect
    //EICRA = (1 << ISC11) | (1 << ISC10) | (RISING << ISC10);
    //EIMSK = (1 << INT1);
    //----------------------------------------------------------------- INT2_vect
    //EICRA = (1 << ISC21) | (1 << ISC20) | (RISING << ISC20);
    //EIMSK = (1 << INT2);
}

void activate_pin_change_int() {
    // Enable pin change interrupt on the PCINT16 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 |= (1 << PCINT16); // PD0 RXD
    // Enable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR |= (1 << PCIE2);    
}

void deactivate_pin_change_int() {
    // Disable pin change interrupt 2 using the Pin Change Interrrupt Control Register (PCICR)
    PCICR &= ~(1 << PCIE2);
    // Enable pin change interrupt on the PCINT16 pin using Pin Change Mask Register 2 (PCMSK2)
    PCMSK2 &= ~(1 << PCINT16); // PD0 RXD
}

// only used to keep track of time while sleeping to adjust millis()
ISR(TIMER2_OVF_vect) { g_timer2Overflows++; }


int main() {
    MCUSR = 0;
    mcu::Watchdog::disable();
    sei();

    mcu::Pin led(PIN_LED_SCK);
    led = 1;
    mcu::Usart &ser = mcu::Usart::get();
    stream::UartStream cout(ser);
    power_adc_disable();
    power_spi_disable();
    power_timer1_disable();
    power_twi_disable(); // managed by I2CMaster::
    
    activate_INT0();
    activate_pin_change_int();
    cout << stream::Flags::PGM << PSTR("not365\r\n");
    _delay_ms(100);
    led = 0;
    _delay_ms(900);
    mcu::Watchdog::enable(WDTO_1S);

    proto.setDebug(g_Debug);
    proto.start();
    uint16_t blink = 0;
    while (1) {
        if (++blink == 0) { proto.print(); }
        
        if (proto.update(led)) g_lastActivity = mcu::Timer::millis();

        if((uint32_t)(mcu::Timer::millis() - g_lastActivity) >= 5000 && !g_Debug) {
            ser.disableTX();
            // go into deep sleep, will wake up every 250ms by BQ769x0 ALERT or from USART1 RX (first byte will be lost)
            cli();
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            // Timer/Counter2 8-byte OVF 8MHz  /1024 = 32.64ms
            // Timer/Counter2 8-byte OVF 12MHz /1024 = 21.76ms            
            TCCR2A = 0;
            TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);
            TCNT2 = 0;
            TIMSK2 = (1<<TOIE2);
            ser.disableRX();
            activate_pin_change_int();
            mcu::Watchdog::reset();
            g_wakeupFlag = false;
            sleep_enable();
            sei();
            // go to sleep if it's just timer2 that woke us up (unless we were idle for longer than 500ms)
            do { sleep_cpu(); } while ( !g_wakeupFlag && g_timer2Overflows < 16 );
            sleep_disable();
            // Disable Timer/Counter2 and add elapsed time to Arduinos 'timer0_millis'
            TCCR2B = 0;
            TIMSK2 = 0;
            //float elapsed_time = g_timer2Overflows * 32.64 + TCNT2 * 32.64 / 255.0;
            float elapsed_time = g_timer2Overflows * 21.76 + TCNT2 * 21.76 / 255.0;
            mcu::Timer::setmillis(mcu::Timer::millis() + (uint32_t)elapsed_time);
            g_timer2Overflows = 0;
            if(g_uartRxInterrupted) g_lastActivity = mcu::Timer::millis();
            g_uartRxInterrupted = false;
            deactivate_pin_change_int();
            ser.enableRX();
            ser.enableTX();
            sei();
        }
        mcu::Watchdog::reset();
    }
}
