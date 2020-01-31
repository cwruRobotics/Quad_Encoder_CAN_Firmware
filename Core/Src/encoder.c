//
// Created by Benjamin Scholar on 1/27/20.
//

#include "encoder.h"

#define BUFFER_LEN 100

// PRIVATE VARIABLES
static __volatile int32_t  _encoder_count      = 0             ;
static __volatile double   _encoder_velocity   = 0.0           ;
static __volatile bool     _encoder_inverted   = false         ;

static __volatile uint8_t  _current_GPIO_value = 0             ;
static __volatile uint8_t  _last_GPIO_value    = 0             ;

static __volatile uint32_t _velocity_time_buffer  [BUFFER_LEN] ;
static __volatile int32_t  _velocity_count_buffer [BUFFER_LEN] ;

static __volatile EncoderStatus _encoder_status = ENCODER_OK   ;

// https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
static const      int8_t quad_encoder_values[]  = {0,-1,1,2,1,0,2,-1,-1,2,0,1,2,1,-1,0};

// FUNCTION IMPLEMENTATIONS
__volatile int32_t * encoder_count(void)        { return &_encoder_count    ; }
__volatile bool    * encoder_inverted(void)     { return &_encoder_inverted ; }

           void      reset_encoder_count(void)  { _encoder_count = 0        ; }

// https://cdn.sparkfun.com/datasheets/Robotics/How%20to%20use%20a%20quadrature%20encoder.pdf
// http://www.ni.com/tutorial/7109/en/
// what encoding sceme should we use? X1, X2, X4? X4 requires 4 times the interrupts as X1
EncoderStatus increment_encoder_from_GPIO(bool a_high, bool b_high) {
    _current_GPIO_value = 0;
    _current_GPIO_value += (a_high << 1u);
    _current_GPIO_value += (b_high << 0u);

    uint8_t increment = quad_encoder_values[_last_GPIO_value * 4 + _current_GPIO_value];
    if(increment == 2) {
        // this is bad.
        // Either we are not reading encoder fast enough, or there's some error
        _encoder_status = ENCODER_COUNT_ERROR;
    } else {
        _encoder_count += (!_encoder_inverted ? increment : -increment);
        _encoder_status = ENCODER_OK;
    }

    _last_GPIO_value = _current_GPIO_value;
    return _encoder_status;
}

EncoderStatus get_encoder_status(void) {
    return _encoder_status;
}



