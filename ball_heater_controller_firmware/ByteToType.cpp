#include "Arduino.h"
#include "ByteToType.h"


float ByteToType::BytesToFloat(byte *inBytes) {
    union u_tag {
        byte b[4];
        float fval;
    } u;
// Flipped endian here so that bytes in the float are same order as bytes in inBytes
    for (int i = 3; i >= 0; i --) {
        u.b[i] = *(inBytes + i);
    }

    return u.fval;
}

// Flipped this endian as well, need to test
uint16_t ByteToType::ByteToUInt16(byte *inBytes) {
    union u_tag {
        byte b[2];
        uint16_t ival;
    } u;

    for (int i = 1; i >= 0; i --) {
        u.b[ i] = *(inBytes + i);
    }

    return u.ival;
}
