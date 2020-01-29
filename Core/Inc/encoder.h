//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_ENCODER_H
#define CANCODER_ENCODER_H

#include <stdbool.h>
#include <inttypes.h>

extern        void    set_encoder_count(int32_t count);
extern inline void    reset_encoder_count(void);
extern        int32_t get_encoder_count(void);
extern        void    set_encoder_inverted(bool inverted);
extern        bool    get_encoder_inverted(void);
extern        void    increment_encoder_from_GPIO(bool a_high, bool b_high);

#endif //CANCODER_ENCODER_H
