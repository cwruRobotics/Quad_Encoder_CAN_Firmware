//
// Created by Benjamin Scholar on 1/29/20.
//

#ifndef CANCODER_UTIL_H
#define CANCODER_UTIL_H

#include <stddef.h>

// memcpy for volatile memory
extern __volatile void* memcpy_v(__volatile void *restrict, const __volatile void *restrict, size_t);

#endif //CANCODER_UTIL_H
