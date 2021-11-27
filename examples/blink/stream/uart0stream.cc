#include "uart0stream.h"

namespace stream {
    
UartStream::UartStream(mcu::Serial0 &Uart):
uart(Uart)
{ }

bool UartStream::avail() { return uart.available(); }

UartStream &UartStream::operator>>(char &ch) {
    ch = uart.read();
    return *this;
}

void UartStream::write(const char ch) {
    uart.write(ch);
}
    
}  // namespace stream
