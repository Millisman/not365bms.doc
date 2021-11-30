#include "serial.h"


#ifndef USART0_RX_vect
#define USART0_RX_vect USART_RX_vect
#endif

// #ifndef UART0_UDRE_vect
// #define UART0_UDRE_vect UART_UDRE_vect
// #endif

#ifndef USART0_UDRE_vect
#define USART0_UDRE_vect USART_UDRE_vect
#endif


#if defined(UBRR0H)

namespace {

volatile rx0_buffer_index_t _rx0_buffer_head;
volatile rx0_buffer_index_t _rx0_buffer_tail;
volatile tx0_buffer_index_t _tx0_buffer_head;
volatile tx0_buffer_index_t _tx0_buffer_tail;
uint8_t _rx0_buffer[SERIAL0_RX_BUFFER_SIZE];
uint8_t _tx0_buffer[SERIAL0_TX_BUFFER_SIZE];

void _rx0_complete_irq(void) {
    if (bit_is_clear(UCSR0A, UPE0)) {
        // No Parity error, read byte and store it in the buffer if there is
        // room
        uint8_t c = UDR0;
        rx0_buffer_index_t i = (uint16_t)(_rx0_buffer_head + 1) % SERIAL0_RX_BUFFER_SIZE;
        // if we should be storing the received character into the location
        // just before the tail (meaning that the head would advance to the
        // current location of the tail), we're about to overflow the buffer
        // and so we don't write the character or advance the head.
        if (i != _rx0_buffer_tail) {
            _rx0_buffer[_rx0_buffer_head] = c;
            _rx0_buffer_head = i;
        }
    } else {
        // Parity error, read byte but discard it
        UDR0;
    };
}

void _tx0_udr_empty_irq(void) {
    // If interrupts are enabled, there must be more data in the output
    // buffer. Send the next byte
    uint8_t c = _tx0_buffer[_tx0_buffer_tail];
    _tx0_buffer_tail = (_tx0_buffer_tail + 1) % SERIAL0_TX_BUFFER_SIZE;
    UDR0 = c;
    // clear the TXC bit -- "can be cleared by writing a one to its bit
    // location". This makes sure flush() won't return until the bytes
    // actually got written. Other r/w bits are preserved, and zeroes
    // written to the rest.
#ifdef MPCM0
    UCSR0A = ((UCSR0A) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
    UCSR0A = ((UCSR0A) & ((1 << U2X0) | (1 << TXC0)));
#endif
    if (_tx0_buffer_head == _tx0_buffer_tail) {
        // Buffer empty, so disable interrupts
        cbi(UCSR0B, UDRIE0);
    }
}

#if defined(USART0_RX_vect)
ISR(USART0_RX_vect)
#else
#error "Don't know what the Data Received vector is called for Serial0"
#endif
{ _rx0_complete_irq(); }

#if defined(UART0_UDRE_vect)
ISR(UART0_UDRE_vect)
#elif defined(USART0_UDRE_vect)
ISR(USART0_UDRE_vect)
#else
#error "Don't know what the Data Register Empty vector is called for Serial0"
#endif
{ _tx0_udr_empty_irq(); }

} // !namespace

namespace mcu {

Serial0::Serial0(uint32_t baud, uint8_t config) {
    _rx0_buffer_head = 0; _rx0_buffer_tail = 0;
    _tx0_buffer_head = 0; _tx0_buffer_tail = 0;
    _baud   = baud;
    _config = config;
    UCSR0A = calc_baud_setting();
    // assign the baud_setting, a.k.a. ubrr (USART Baud Rate Register)
    UBRR0H = _baud_setting >> 8;
    UBRR0L = _baud_setting;
    SERIAL0_WRITTEN(_written = false;)
    //set the data bits, parity, and stop bits
#if defined(__AVR_ATmega8__)
    config |= 0x80; // select UCSRC register (shared with UBRRH)
#endif
    UCSR0C = config;
    sbi(UCSR0B, RXEN0);
    sbi(UCSR0B, TXEN0);
    sbi(UCSR0B, RXCIE0);
    cbi(UCSR0B, UDRIE0);
}

void Serial0::end() {
    // wait for transmission of outgoing data
    flush();
    cbi(UCSR0B, RXEN0);
    cbi(UCSR0B, TXEN0);
    cbi(UCSR0B, RXCIE0);
    cbi(UCSR0B, UDRIE0);
    // clear any received data
    _rx0_buffer_head = _rx0_buffer_tail;
}

int16_t Serial0::available(void) {
    return ((uint16_t)(SERIAL0_RX_BUFFER_SIZE + _rx0_buffer_head - _rx0_buffer_tail)) % SERIAL0_RX_BUFFER_SIZE;
}

int16_t Serial0::peek(void) {
    if (_rx0_buffer_head == _rx0_buffer_tail) { return -1; }
    else { return _rx0_buffer[_rx0_buffer_tail]; }
}

int16_t Serial0::read(void) {
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx0_buffer_head == _rx0_buffer_tail) { return -1; }
    else {
        uint8_t c = _rx0_buffer[_rx0_buffer_tail];
        _rx0_buffer_tail = (rx0_buffer_index_t)(_rx0_buffer_tail + 1) % SERIAL0_RX_BUFFER_SIZE;
        return c;
    }
}

int16_t Serial0::availableForWrite(void) {
    tx0_buffer_index_t head;
    tx0_buffer_index_t tail;
    TX0_BUFFER_ATOMIC {
        head = _tx0_buffer_head;
        tail = _tx0_buffer_tail;
    }
    if (head >= tail) return SERIAL0_TX_BUFFER_SIZE - 1 - head + tail;
    return tail - head - 1;
}

void Serial0::flush() {
    // If we have never written a byte, no need to flush. This special
    // case is needed since there is no way to force the TXC (transmit
    // complete) bit to 1 during initialization
    SERIAL0_WRITTEN( if (!_written) return;)
    while (bit_is_set(UCSR0B, UDRIE0) || bit_is_clear(UCSR0A, TXC0)) {
        if (bit_is_clear(SREG, SREG_I) && bit_is_set(UCSR0B, UDRIE0))
            // Interrupts are globally disabled, but the DR empty
            // interrupt should be enabled, so poll the DR empty flag to
            // prevent deadlock
            if (bit_is_set(UCSR0A, UDRE0))
                _tx0_udr_empty_irq();
    }
    // If we get here, nothing is queued anymore (DRIE is disabled) and
    // the hardware finished tranmission (TXC is set).
}

size_t Serial0::write(uint8_t c) {
    SERIAL0_WRITTEN(_written = true;)
    // If the buffer and the data register is empty, just write the byte
    // to the data register and be done. This shortcut helps
    // significantly improve the effective datarate at high (>
    // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
    if (_tx0_buffer_head == _tx0_buffer_tail && bit_is_set(UCSR0A, UDRE0)) {
        // If TXC is cleared before writing UDR and the previous byte
        // completes before writing to UDR, TXC will be set but a byte
        // is still being transmitted causing flush() to return too soon.
        // So writing UDR must happen first.
        // Writing UDR and clearing TC must be done atomically, otherwise
        // interrupts might delay the TXC clear so the byte written to UDR
        // is transmitted (setting TXC) before clearing TXC. Then TXC will
        // be cleared when no bytes are left, causing flush() to hang
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            UDR0 = c;
#ifdef MPCM0
            UCSR0A = ((UCSR0A) & ((1 << U2X0) | (1 << MPCM0))) | (1 << TXC0);
#else
            UCSR0A = ((UCSR0A) & ((1 << U2X0) | (1 << TXC0)));
#endif
        }
        return 1;
    }
    tx0_buffer_index_t i = (_tx0_buffer_head + 1) % SERIAL0_TX_BUFFER_SIZE;
    // If the output buffer is full, there's nothing for it other than to 
    // wait for the interrupt handler to empty it a bit
    while (i == _tx0_buffer_tail) {
        if (bit_is_clear(SREG, SREG_I)) {
            // Interrupts are disabled, so we'll have to poll the data
            // register empty flag ourselves. If it is set, pretend an
            // interrupt has happened and call the handler to free up
            // space for us.
            if(bit_is_set(UCSR0A, UDRE0)) _tx0_udr_empty_irq();
        } else {
            // nop, the interrupt handler will free up space for us
        }
    }
    _tx0_buffer[_tx0_buffer_head] = c;
    // make atomic to prevent execution of ISR between setting the
    // head pointer and setting the interrupt flag resulting in buffer
    // retransmission
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        _tx0_buffer_head = i;
        sbi(UCSR0B, UDRIE0);
    }
    return 1;
}

} // !namespace mcu

#endif // !defined(UBRR0H)
