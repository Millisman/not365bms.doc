#include "dump.h"
#include "bcd.h"

namespace utils {

void Hexdump(stream::OutputStream& out, const uint8_t RowSize, bool ShowAscii, const void* mData, const uint8_t mLength) {
    uint8_t n;
    const char sep[] = " | ";
    for (uint8_t i = 0; i < mLength; i += RowSize) {
        n = i;
        out << "0x" << ToHex(n) << sep;

        for (uint8_t j = 0; j < RowSize; ++j) {
            if (i + j < mLength) {
                n = ((uint8_t*)mData)[i + j];
                out << ToHex(n);
            } else out << "   ";
        }

        if (ShowAscii) {
            out << sep;
            for (uint8_t j = 0; j < RowSize; ++j) {
                if (i + j < mLength) {
                    if (((uint8_t*)mData)[i + j] >= ' ' && ((uint8_t*)mData)[i + j] <= '~') {
                        out << ((char*)mData)[i + j];
                    } else out << '_';
                }
            }
        }
        out << EOL;
    }
}
    
}


