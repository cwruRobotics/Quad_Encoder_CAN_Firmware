//
// Created by Benjamin Scholar on 1/27/20.
//

#include "encoder.h"

__volatile int32_t encoder_count    = 0     ;
__volatile double  encoder_velocity = 0.0d  ;
__volatile bool    encoder_reversed = false ;