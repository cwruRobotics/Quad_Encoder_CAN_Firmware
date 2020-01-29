//
// Created by Benjamin Scholar on 1/27/20.
//

#include "encoder.h"

#define BUFFER_LENGTH 100

// PRIVATE VARIABLES
static __volatile int32_t encoder_count    = 0     ;
static __volatile double  encoder_velocity = 0.0   ; // we wont use this for now
static __volatile bool    encoder_inverted = false ;

static __volatile uint8_t current_GPIO_value = 0 ;
static __volatile uint8_t last_GPIO_value    = 0 ;

static __volatile EncoderStatus encoder_status = ENCODER_OK ;

// buffers for measuring velocity
static const      int32_t encoder_count_buffer [BUFFER_LENGTH];
static const      int32_t encoder_time_buffer  [BUFFER_LENGTH];

// https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
static const      uint8_t quad_encoder_values[] = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

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

// https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
// http://www.ni.com/tutorial/7109/en/
// what encoding scene should we use? X1, X2, X4? X4 requires 4 times the interrupts as X1
EncoderStatus increment_encoder_from_GPIO(bool a_high, bool b_high) {
    current_GPIO_value = 0;
    current_GPIO_value += a_high << 1;
    current_GPIO_value += b_high << 0;

    uint8_t increment = quad_encoder_values[last_GPIO_value * 4 + current_GPIO_value];
    if(increment == 2) {
        // this is bad.
        // Either we are not reading encoder fast enough, or there's some error
        encoder_status = ENCODER_COUNT_ERROR;
    } else {
        encoder_count += (!encoder_inverted ? increment : -increment);
        encoder_status = ENCODER_OK;
    }

    last_GPIO_value = current_GPIO_value;
    return encoder_status;
}

EncoderStatus get_encoder_status() {
    return encoder_status;
}



