#pragma once

#include <stdint.h>
#include "utils/cpp.h"

namespace mcu {
    
class I2CMaster {
    I2CMaster();
    
public:
    static I2CMaster &get();
    
    void write(const uint8_t addr, uint8_t *data, const uint8_t len);
    void read(const uint8_t addr, uint8_t *data, const uint8_t len);
    
private:
    DISALLOW_COPY_AND_ASSIGN(I2CMaster);
};
    
}
