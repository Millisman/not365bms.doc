#pragma once

#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <utils/cpp.h>





#define  SERIAL_5N1  0x00
#define  SERIAL_6N1  0x02
#define  SERIAL_7N1  0x04
#define  SERIAL_8N1  0x06
#define  SERIAL_5N2  0x08
#define  SERIAL_6N2  0x0A
#define  SERIAL_7N2  0x0C
#define  SERIAL_8N2  0x0E
#define  SERIAL_5E1  0x20
#define  SERIAL_6E1  0x22
#define  SERIAL_7E1  0x24
#define  SERIAL_8E1  0x26
#define  SERIAL_5E2  0x28
#define  SERIAL_6E2  0x2A
#define  SERIAL_7E2  0x2C
#define  SERIAL_8E2  0x2E
#define  SERIAL_5O1  0x30
#define  SERIAL_6O1  0x32
#define  SERIAL_7O1  0x34
#define  SERIAL_8O1  0x36
#define  SERIAL_5O2  0x38
#define  SERIAL_6O2  0x3A
#define  SERIAL_7O2  0x3C
#define  SERIAL_8O2  0x3E



#ifndef SERIAL0_BAUD
#define SERIAL0_BAUD 115200
#endif

#ifndef SERIAL1_BAUD
#define SERIAL1_BAUD 57600
#endif

#ifndef SERIAL2_BAUD
#define SERIAL2_BAUD 9600
#endif

#ifndef SERIAL3_BAUD
#define SERIAL3_BAUD 9600
#endif


#ifndef SERIAL0_CONFIG
#define SERIAL0_CONFIG SERIAL_8N1
#endif

#ifndef SERIAL1_CONFIG
#define SERIAL1_CONFIG SERIAL_8N1
#endif

#ifndef SERIAL2_CONFIG
#define SERIAL2_CONFIG SERIAL_8N1
#endif

#ifndef SERIAL3_CONFIG
#define SERIAL3_CONFIG SERIAL_8N1
#endif

#ifndef SERIAL0_TX_BUFFER_SIZE
#define SERIAL0_TX_BUFFER_SIZE 64
#endif

#ifndef SERIAL0_RX_BUFFER_SIZE
#define SERIAL0_RX_BUFFER_SIZE 64
#endif

#ifndef SERIAL1_TX_BUFFER_SIZE
#define SERIAL1_TX_BUFFER_SIZE 64
#endif

#ifndef SERIAL1_RX_BUFFER_SIZE
#define SERIAL1_RX_BUFFER_SIZE 64
#endif

#ifndef SERIAL2_TX_BUFFER_SIZE
#define SERIAL2_TX_BUFFER_SIZE 64
#endif

#ifndef SERIAL2_RX_BUFFER_SIZE
#define SERIAL2_RX_BUFFER_SIZE 64
#endif

#ifndef SERIAL3_TX_BUFFER_SIZE
#define SERIAL3_TX_BUFFER_SIZE 64
#endif

#ifndef SERIAL3_RX_BUFFER_SIZE
#define SERIAL3_RX_BUFFER_SIZE 64
#endif

#define SERIAL0_WRITTEN(x) x
// #define SERIAL0_WRITTEN(x)
#define SERIAL1_WRITTEN(x) x
// #define SERIAL1_WRITTEN(x)
#define SERIAL2_WRITTEN(x) x
// #define SERIAL2_WRITTEN(x)
#define SERIAL3_WRITTEN(x) x
// #define SERIAL3_WRITTEN(x)

#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#define bit(b) (1UL << (b))


namespace mcu {

class HardwareSerial {
public:
    uint8_t  _config;
    uint32_t _baud;
    uint16_t _baud_setting;
    //set the data bits, parity, and stop bits
    // hardcoded exception for 57600 for compatibility with the bootloader
    // shipped with the Duemilanove and previous boards and the firmware
    // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
    // be > 4095, so switch back to non-u2x mode if the baud rate is too
    // low.
    HardwareSerial() {}
    virtual ~HardwareSerial() {}
    HardwareSerial &operator>>(uint8_t &ch) { ch = read(); return *this; }    
    HardwareSerial &operator<<(const uint8_t &ch) { write(ch); return *this; }
    HardwareSerial &operator<<(const char *str) {while (*str) write(*str++); return *this; }
    
    // wait for transmission of outgoing data
    // clear any received data
    virtual void end() = 0;
    
    virtual int16_t available(void) = 0;
    
    virtual int16_t peek(void) = 0;
    // if the head isn't ahead of the tail, we don't have any characters
    virtual int16_t read(void) = 0;
    
    virtual int16_t availableForWrite(void) = 0;
    
    // If we have never written a byte, no need to flush. This special
    // case is needed since there is no way to force the TXC (transmit
    // complete) bit to 1 during initialization
    // Interrupts are globally disabled, but the DR empty
    // interrupt should be enabled, so poll the DR empty flag to
    // prevent deadlock
    // If we get here, nothing is queued anymore (DRIE is disabled) and
    // the hardware finished tranmission (TXC is set).
    virtual void flush(void) = 0;
    
    // If the buffer and the data register is empty, just write the byte
    // to the data register and be done. This shortcut helps
    // significantly improve the effective datarate at high (>
    // 500kbit/s) bitrates, where interrupt overhead becomes a slowdown.
    // If TXC is cleared before writing UDR and the previous byte
    // completes before writing to UDR, TXC will be set but a byte
    // is still being transmitted causing flush() to return too soon.
    // So writing UDR must happen first.
    // Writing UDR and clearing TC must be done atomically, otherwise
    // interrupts might delay the TXC clear so the byte written to UDR
    // is transmitted (setting TXC) before clearing TXC. Then TXC will
    // be cleared when no bytes are left, causing flush() to hang
    // If the output buffer is full, there's nothing for it other than to 
    // wait for the interrupt handler to empty it a bit
    // make atomic to prevent execution of ISR between setting the
    // head pointer and setting the interrupt flag resulting in buffer
    // retransmission
    virtual size_t write(uint8_t) = 0;
    
    uint8_t calc_baud_setting() {
        // Try u2x mode first
        _baud_setting = (F_CPU / 4 / _baud - 1) / 2;
        uint8_t UCSRxA = 1 << U2X0;
        // hardcoded exception for 57600 for compatibility with the bootloader
        // shipped with the Duemilanove and previous boards and the firmware
        // on the 8U2 on the Uno and Mega 2560. Also, The baud_setting cannot
        // be > 4095, so switch back to non-u2x mode if the baud rate is too
        // low.
        if (((F_CPU == 16000000UL) && (_baud == 57600)) || (_baud_setting > 4095)) {
            UCSRxA = 0;
            _baud_setting = (F_CPU / 8 / _baud - 1) / 2;
        }
        return UCSRxA;
    }
    
protected:

};

} // !namespace mcu

#if defined(UBRR0H)

#if (SERIAL0_TX_BUFFER_SIZE>256)
typedef uint16_t tx0_buffer_index_t;
#else
typedef uint8_t  tx0_buffer_index_t;
#endif

#if  (SERIAL0_RX_BUFFER_SIZE>256)
typedef uint16_t rx0_buffer_index_t;
#else
typedef uint8_t  rx0_buffer_index_t;
#endif

#if (SERIAL0_TX_BUFFER_SIZE>256)
#define TX0_BUFFER_ATOMIC ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
#else
#define TX0_BUFFER_ATOMIC
#endif

namespace mcu {

class Serial0 : public HardwareSerial {
public:
    Serial0(uint32_t baud, uint8_t config);
    static Serial0 &init() {
        static Serial0 usart(SERIAL0_BAUD, SERIAL0_CONFIG);
        return usart;
    }
    void end() override;
    int16_t available(void) override;
    int16_t peek(void) override;
    int16_t read(void) override;
    int16_t availableForWrite(void) override;
    void flush(void) override;
    size_t write(uint8_t) override;
private:
    DISALLOW_COPY_AND_ASSIGN(Serial0);
protected:
    SERIAL0_WRITTEN(bool _written;)
};

} // !namespace mcu

#endif // !defined(UBRR0H)


