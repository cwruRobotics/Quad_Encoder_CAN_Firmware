//
// Created by Benjamin Scholar on 1/27/20.
//

#include "encoder.h"

// PRIVATE VARIABLES
static __volatile int32_t encoder_count    = 0     ;
static __volatile double  encoder_velocity = 0.0   ; // we wont use this for now
static __volatile bool    encoder_inverted = false ;


// FUNCTION IMPLEMENTATIONS
void set_encoder_count(int32_t count) {
    encoder_count = count;
}

inline void reset_encoder_count(){
    set_encoder_count(0);
}

int32_t get_encoder_count() {
    return encoder_count;
}

void set_encoder_inverted(bool inverted) {
    encoder_inverted = inverted;
}

bool get_encoder_inverted() {
    return encoder_inverted;
}

void increment_encoder_from_GPIO(bool a_high, bool b_high) {
    // TODO(Ben): Implement this
}



