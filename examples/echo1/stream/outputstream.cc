#include "outputstream.h"

#include <avr/pgmspace.h>
#include <stdlib.h>

namespace stream {

OutputStream::OutputStream() : flags(0) {}

OutputStream &OutputStream::operator<<(const Flags flag) {
  flags |= (1 << flag);
  return *this;
}

OutputStream &OutputStream::operator<<(const Spaces spaces) {
  for (size_t i = 0; i != spaces.num; ++i) *this << ' ';
  return *this;
}

OutputStream &OutputStream::operator<<(const char ch) {
  write(ch);
  return *this;
}

/*
OutputStream &OutputStream::operator<<(const FlashStringHelper *str) {
    PGM_P p = reinterpret_cast<PGM_P>(ifsh);
//     if (flags & (1 << Flags::PGM)) {
    while (pgm_read_byte_far(str)) write(pgm_read_byte_far(str++));
//     } else {
//         while (*str) write(*str++);
//     }
    flags = 0;
    return *this;
}*/





OutputStream &OutputStream::operator<<(const char *str) {
  if (flags & (1 << Flags::PGM)) {
//       while (pgm_read_byte_far(str)) write(pgm_read_byte_far(str++));
      while (pgm_read_byte_near(str)) write(pgm_read_byte_near(str++));
  } else {
    while (*str) write(*str++);
  }
  flags = 0;
  return *this;
}

OutputStream &OutputStream::operator<<(const uint8_t val) {
  if (flags == 0) {
    char buffer[4] = {0};
    utoa(val, buffer, 10);
    return *this << buffer;
  }

  else if (flags & ((1 << Flags::PAD_ZERO) | (1 << Flags::PAD_SPACE))) {
    if (val < 10)
      *this << static_cast<char>((flags & (1 << Flags::PAD_ZERO)) ? '0' : ' ');
    else
      *this << static_cast<char>('0' + val / 10);
    *this << static_cast<char>('0' + val % 10);
  }

  flags = 0;
  return *this;
}

OutputStream &OutputStream::operator<<(const int8_t val) {
    if (flags == 0) {
        char buffer[5] = {0};
        itoa(val, buffer, 10);
        return *this << buffer;
    }
    
    else if (flags & ((1 << Flags::PAD_ZERO) | (1 << Flags::PAD_SPACE))) {
        if (val < 10)
            *this << static_cast<char>((flags & (1 << Flags::PAD_ZERO)) ? '0' : ' ');
        else
            *this << static_cast<char>('0' + val / 10);
        *this << static_cast<char>('0' + val % 10);
    }
    
    flags = 0;
    return *this;
}



OutputStream &OutputStream::operator<<(const uint16_t val) {
    if (flags == 0) {
        char buffer[7] = {0};
        utoa(val, buffer, 10);
        return *this << buffer;
    }
    
    else if (flags & ((1 << Flags::PAD_ZERO) | (1 << Flags::PAD_SPACE))) {
        if (val < 10)
            *this << static_cast<char>((flags & (1 << Flags::PAD_ZERO)) ? '0' : ' ');
        else
            *this << static_cast<char>('0' + val / 10);
        *this << static_cast<char>('0' + val % 10);
    }
    
    flags = 0;
    return *this;
}


OutputStream &OutputStream::operator<<(const uint32_t val) {
  char buffer[11] = {0};
  ltoa(val, buffer, 10);
  return *this << buffer;
}

}  // namespace stream
