//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_ENCODER_H
#define CANCODER_ENCODER_H

#include <stdbool.h>
#include <inttypes.h>

extern __volatile int32_t encoder_count    ;
extern __volatile double  encoder_velocity ; // we wont use this for now
extern __volatile bool    encoder_reversed ;

#endif //CANCODER_ENCODER_H
