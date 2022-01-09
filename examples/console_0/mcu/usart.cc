#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>
#include "mcu/usart.h"
#include "utils/atomic.h"


#define USART0_RX_BUFFER_SIZE 128
#define USART0_TX_BUFFER_SIZE 128

namespace {
    
    static volatile uint8_t USART0_RX_BUFFER[USART0_RX_BUFFER_SIZE];
    static volatile uint8_t USART0_RX_BUFFER_HEAD;
    static volatile uint8_t USART0_RX_BUFFER_TAIL;
    
    static volatile uint8_t USART0_TX_BUFFER[USART0_TX_BUFFER_SIZE];
    static volatile uint8_t USART0_TX_BUFFER_HEAD;
    static volatile uint8_t USART0_TX_BUFFER_TAIL;
    
    ISR(USART_RX_vect) {
        if (bit_is_set(UCSR0A, UPE0)) {
            UDR0;
        } else {
            uint8_t c = UDR0;
            uint8_t i = (USART0_RX_BUFFER_HEAD + 1 >= USART0_RX_BUFFER_SIZE) ? 0 : USART0_RX_BUFFER_HEAD + 1;
            if (i != USART0_RX_BUFFER_TAIL) {
                USART0_RX_BUFFER[USART0_RX_BUFFER_HEAD] = c;
                USART0_RX_BUFFER_HEAD = i;
            }
        }
    }
    
    ISR(USART_TX_vect) {
        if (USART0_TX_BUFFER_HEAD != USART0_TX_BUFFER_TAIL) {
            uint8_t c = USART0_TX_BUFFER[USART0_TX_BUFFER_TAIL];
            if (++USART0_TX_BUFFER_TAIL >= USART0_TX_BUFFER_SIZE) USART0_TX_BUFFER_TAIL = 0;  // хвост двигаем
            UDR0 = c;
        }
    }
    
}


namespace mcu {
    
    Usart &Usart::get() {
        static Usart usart(115200, (1<<UCSZ01) | (1<<UCSZ00));
        return usart;
    }
    
    Usart::Usart(uint32_t baud, uint8_t config)
    {
        USART0_RX_BUFFER_HEAD = 0;
        USART0_RX_BUFFER_TAIL = 0;
        USART0_TX_BUFFER_HEAD = 0;
        USART0_TX_BUFFER_TAIL = 0;
        // Try u2x mode first
        uint16_t baud_setting = (F_CPU / 4 / baud - 1) / 2;
        UCSR0A = 1 << U2X0;
        // hardcoded exception for 57600 for compatibility with the bootloader
        // shipped with the Duemilanove and previous boards and the firmware
        // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
        // be > 4095, so switch back to non-u2x mode if the baud rate is too
        // low.
        //         if (((F_CPU == 16000000UL) && (baud == 57600)) || (baud_setting >4095))
        //         {
        //             UCSR0A = 0;
        //             baud_setting = (F_CPU / 8 / baud - 1) / 2;
        //         }
        
        // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
        UBRR0H = baud_setting >> 8;
        UBRR0L = baud_setting;
        UCSR0B = ((1<<TXEN0) | (1<<RXEN0) | (1<<RXCIE0) | (1<<TXCIE0));
        UCSR0C = config; //((1<<UCSZ01) | (1<<UCSZ00));
    }
    
//     void Usart::end(){
//         UCSR0B = 0;
//     }
    
    
    uint8_t Usart::read() {
        if (USART0_RX_BUFFER_HEAD == USART0_RX_BUFFER_TAIL) return -1;
        uint8_t c = USART0_RX_BUFFER[USART0_RX_BUFFER_TAIL];
        if (++USART0_RX_BUFFER_TAIL >= USART0_RX_BUFFER_SIZE) USART0_RX_BUFFER_TAIL = 0;  // хвост двигаем
        return c;
    }
    
//     uint8_t Usart::peek() {
//         return USART0_RX_BUFFER_HEAD != USART0_RX_BUFFER_TAIL ? USART0_RX_BUFFER[USART0_RX_BUFFER_TAIL]: -1;
//     }
    
//     void Usart::flush() {
//         while (USART0_RX_BUFFER_HEAD != USART0_RX_BUFFER_TAIL);
//     }
    
    uint16_t Usart::avail() {
        return ((uint16_t)(USART0_RX_BUFFER_SIZE + USART0_RX_BUFFER_HEAD - USART0_RX_BUFFER_TAIL)) % USART0_RX_BUFFER_SIZE;
    } 
    
    
//     bool Usart::avail() {
//         return available(); 
//     }
//     
//     
//     void Usart::clear() {
//         USART0_RX_BUFFER_HEAD = USART0_RX_BUFFER_TAIL = 0;
//     }
    
    
    void Usart::write(const uint8_t data) {
        uint8_t i = (USART0_TX_BUFFER_HEAD + 1 >= USART0_TX_BUFFER_SIZE) ? 0 : USART0_TX_BUFFER_HEAD + 1;
        // ждать освобождения места в буфере
        while ( (i + 1) == USART0_TX_BUFFER_TAIL);
        
        // Не сохранять новые данные если нет места
        if (i != USART0_TX_BUFFER_TAIL) {
            USART0_TX_BUFFER[USART0_TX_BUFFER_HEAD] = data;
            USART0_TX_BUFFER_HEAD = i;
        }
        while (!(UCSR0A & (1<<UDRE0)));
        if (USART0_TX_BUFFER_HEAD != USART0_TX_BUFFER_TAIL) {
            uint8_t c = USART0_TX_BUFFER[USART0_TX_BUFFER_TAIL];
            if (++USART0_TX_BUFFER_TAIL >= USART0_TX_BUFFER_SIZE) USART0_TX_BUFFER_TAIL = 0;  // хвост двигаем
            UDR0 = c;
        }
    }
    
}  // namespace mcu








// #include "mcu/usart.h"
// 
// #define USART_BAUD_RATE 115200
// 
// #define USART_BUFFER_SIZE 280
// 
// #define USART_BAUD_PRESCALE (((F_CPU / USART_BAUD_RATE) / 8) - 1)
// 
// namespace {
// 
// volatile uint8_t usart_buffer[USART_BUFFER_SIZE] = {0};
// volatile uint16_t usart_index_insert = 0;
// volatile uint16_t usart_index_read = 0;
// 
// void usart_insert(const uint8_t b) {
//     usart_buffer[usart_index_insert++] = b;
//     if (usart_index_insert == USART_BUFFER_SIZE) usart_index_insert = 0;
//     if (usart_index_insert == usart_index_read) ++usart_index_read;
// }
// 
// uint8_t usart_read() {
//     utils::Atomic _atomic;
//     if (usart_index_read >= USART_BUFFER_SIZE) usart_index_read = 0;
//     if (usart_index_read == usart_index_insert) return -1;
//     return usart_buffer[usart_index_read++];
// }
// 
// uint16_t usart_available() {
//     utils::Atomic _atomic;
//     return usart_index_insert - usart_index_read;
// }
// 
// ISR(USART_RX_vect) { usart_insert(UDR0); }
// 
// }
// 
// namespace mcu {
//     
// Usart &Usart::get() {
//     static Usart usart;
//     return usart;
// }
// 
// Usart::Usart() {
//     UBRR0H = USART_BAUD_PRESCALE >> 8;
//     UBRR0L = USART_BAUD_PRESCALE & 0xFF;
//     UCSR0A |= (1 << U2X0);
//     UCSR0B = (1 << RXCIE0);
//     
//     
// //     UCSR0B |= (1 << RXEN0);
// //     UCSR0B |= (1 << TXEN0);
//     
//     enableRX();
//     enableTX();
//     UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
// }
// 
// void Usart::write(const uint8_t b) {
//     while (0 == (UCSR0A & (1 << UDRE0))) {}
//     UDR0 = b;
// }
// 
// uint8_t Usart::read()   { return usart_read(); }
// uint16_t Usart::avail() { return usart_available(); }
//     
// }
