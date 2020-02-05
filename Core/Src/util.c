//
// Created by Benjamin Scholar on 1/29/20.
//

#include "util.h"

#include "stm32f1xx_hal.h"

__volatile void* memcpy_v(__volatile void *restrict dest, const __volatile void *restrict src, size_t n) {
    const __volatile unsigned char *src_c = src;
    __volatile unsigned char *dest_c      = dest;

    while(n > 0) {
        n--;
        dest_c[n] = src_c[n];
    }
    return dest;
}

uint32_t switch_endian_u32(uint32_t x) {
    uint32_t b0, b1, b2, b3;
    uint32_t res;

    b0 = (x & 0x000000FF) << 24u ;
    b1 = (x & 0x0000FF00) <<  8u ;
    b2 = (x & 0x00FF0000) >>  8u ;
    b3 = (x & 0xFF000000) >> 24u ;

    res = b0 | b1 | b2 | b3;
    return res;
}

int32_t  switch_endian_i32(int32_t x)  {
    int32_t res;
    int32_t b0, b1, b2, b3;

    b0 = (x & 0x000000FF) << 24u ;
    b1 = (x & 0x0000FF00) <<  8u ;
    b2 = (x & 0x00FF0000) >>  8u ;
    b3 = (x & 0xFF000000) >> 24u ;

    res = b0 | b1 | b2 | b3;
    return res;
}

uint16_t switch_endian_u16(uint16_t x) {
    uint16_t b0, b1;
    uint16_t res;

    b0 = (x & 0x00FF) << 8;
    b1 = (x & 0xFF00) >> 8;

    res = b0 | b1;
    return res;
}

int16_t switch_endian_i16(int16_t x ) {
    int16_t b0, b1;
    int16_t res;

    b0 = (x & 0x00FF) << 8;
    b1 = (x & 0xFF00) >> 8;

    res = b0 | b1;
    return res;
}

void DWT_Init(void) {
    if(!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    }
}

void DWT_Delay(uint32_t us) {
    uint32_t startTick = DWT->CYCCNT;
    uint32_t delayTicks = us * (SystemCoreClock / 1000000);
    while (DWT->CYCCNT - startTick < delayTicks);
}
