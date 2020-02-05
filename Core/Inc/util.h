//
// Created by Benjamin Scholar on 1/29/20.
//

#ifndef CANCODER_UTIL_H
#define CANCODER_UTIL_H

#include <stddef.h>
#include <stdint.h>

// memcpy for volatile memory
extern __volatile void* memcpy_v(__volatile void *restrict, const __volatile void *restrict, size_t);

// endian conversion
extern uint32_t switch_endian_u32(uint32_t);
extern int32_t  switch_endian_i32(int32_t );

extern uint16_t little_to_big_u16(uint16_t);
extern uint16_t big_to_little_u16(uint16_t);
extern int16_t  little_to_big_i16(int16_t );

// microsecond sleep
extern void DWT_Init(void);
extern void DWT_Delay(uint32_t us);

#endif //CANCODER_UTIL_H
