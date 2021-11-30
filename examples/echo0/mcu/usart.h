#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "utils/cpp.h"

namespace mcu {
    
class Usart {
public:
    static Usart &get();
    
    uint8_t read();
    void write(const uint8_t b);
    
    bool avail() const;
    
private:
    Usart();
    
    DISALLOW_COPY_AND_ASSIGN(Usart);
};

}
