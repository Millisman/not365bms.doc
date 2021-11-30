#include <avr/interrupt.h>
#include <avr/io.h>
#include <stdint.h>

#include "mcu/usart.h"
#include "utils/atomic.h"

#define USART_BAUD_RATE 115200

#define USART_BUFFER_SIZE 128

#define USART_BAUD_PRESCALE (((F_CPU / USART_BAUD_RATE) / 8) - 1)

namespace {
volatile uint8_t usart_buffer[USART_BUFFER_SIZE] = {0};
volatile uint8_t usart_index_insert = 0;
volatile uint8_t usart_index_read = 0;

void usart_insert(const uint8_t b) {
    usart_buffer[usart_index_insert++] = b;
    if (usart_index_insert == USART_BUFFER_SIZE) usart_index_insert = 0;
    if (usart_index_insert == usart_index_read) ++usart_index_read;
}

uint8_t usart_read() {
    utils::Atomic _atomic;
    if (usart_index_read >= USART_BUFFER_SIZE) usart_index_read = 0;
    if (usart_index_read == usart_index_insert) return -1;
    return usart_buffer[usart_index_read++];
}

uint8_t usart_available() {
    utils::Atomic _atomic;
    return usart_index_insert - usart_index_read;
}

ISR(USART_RX_vect) { usart_insert(UDR0); }
}

namespace mcu {
    
Usart &Usart::get() {
    static Usart usart;
    return usart;
}

Usart::Usart() {
    UBRR0H = USART_BAUD_PRESCALE >> 8;
    UBRR0L = USART_BAUD_PRESCALE & 0xFF;
    UCSR0A |= (1 << U2X0);
    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
}

void Usart::write(const uint8_t b) {
    while (0 == (UCSR0A & (1 << UDRE0))) {}
    UDR0 = b;
}

uint8_t Usart::read() { return usart_read(); }
bool Usart::avail() const { return usart_available(); }
    
}
