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
