//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_ENCODER_H
#define CANCODER_ENCODER_H

#include <stdbool.h>
#include <inttypes.h>

#include "types.h"

extern __volatile int32_t * encoder_count(void);
extern __volatile bool    * encoder_inverted(void);

extern            void      reset_encoder_count(void);

extern EncoderStatus increment_encoder_from_GPIO(bool, bool);
extern EncoderStatus get_encoder_status(void);

#endif //CANCODER_ENCODER_H
