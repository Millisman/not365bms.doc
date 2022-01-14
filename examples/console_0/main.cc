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
#include "protocol/console.h"

#define PIN_LED_SCK MAKEPIN(B, 5, OUT)

#define CHANGE  1
#define FALLING 2
#define RISING  3

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

static volatile bool isrWU = false;
static volatile bool isrRX = false;
static volatile uint16_t timer2ovf = 0;

// only used to keep track of time while sleeping to adjust millis()
ISR(TIMER2_OVF_vect) { timer2ovf++; }
// ISR(INT1_vect)
// ISR(INT2_vect)
ISR(INT0_vect)   { isrWU = true; }
ISR(PCINT2_vect) { isrRX = true; }

int main() {
    MCUSR = 0;
    mcu::Watchdog::disable();
    sei();
    mcu::Pin led(PIN_LED_SCK);   
    led = 1;
    protocol::Console proto; // device protocol class
    mcu::Usart &ser = mcu::Usart::get();
    power_adc_disable();
    power_spi_disable();
    power_timer1_disable();
    power_twi_disable(); // managed by I2CMaster::    
    activate_INT0();
    _delay_ms(100);
    led = 0;
    _delay_ms(900);
    mcu::Watchdog::enable(WDTO_4S);
    proto.begin();
    uint32_t last_Activity = 0;
    uint32_t rrr = 1;
    while (1) {
        if (ser.isActivity() || proto.update(led, isrWU) || proto.Recv()) {
            last_Activity = mcu::Timer::millis();
        } else rrr++;
        
        isrWU = false;
        if((uint32_t)(mcu::Timer::millis() - last_Activity) >= 200000) {
            ser.disable_TXRx();
            cli();
            set_sleep_mode(SLEEP_MODE_PWR_SAVE);
            // Timer/Counter2 8-byte OVF 8MHz  /1024 = 32.64ms
            // Timer/Counter2 8-byte OVF 12MHz /1024 = 21.76ms            
            TCCR2A = 0;
            TCCR2B = (1<<CS22)|(1<<CS21)|(1<<CS20);
            TCNT2 = 0;
            TIMSK2 = (1<<TOIE2);
            activate_pin_change_int();
            mcu::Watchdog::reset();
            sleep_enable();
            sei();
            isrWU = false;
            isrRX = false;
            do {
                sleep_cpu();
            } while (!isrWU && !isrRX && (timer2ovf < 20));
            sleep_disable();
            deactivate_pin_change_int();
            ser.enable_TxRx();
            // Disable Timer/Counter2 and add elapsed time to Arduinos 'timer0_millis'
            TCCR2B = 0;
            TIMSK2 = 0;
            //float elapsed_time = timer2ovf * 32.64 + TCNT2 * 32.64 / 255.0;
            float elapsed_time = timer2ovf * 21.76 + TCNT2 * 21.76 / 255.0;
            mcu::Timer::setmillis(mcu::Timer::millis() + (uint32_t)elapsed_time);
            timer2ovf = 0;
            if(isrRX) last_Activity = mcu::Timer::millis();
            isrRX = false;
            isrWU = true; // forcing CC
        }
        if (rrr == 0x7FFFF) { isrWU = true; rrr = 0; } // forcing CC
        mcu::Watchdog::reset();
    }
}
