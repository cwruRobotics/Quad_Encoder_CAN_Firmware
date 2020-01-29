//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_ENCODER_H
#define CANCODER_ENCODER_H

#include <stdbool.h>
#include <inttypes.h>

#include "types.h"

extern void          set_encoder_count(int32_t count);
extern void          reset_encoder_count(void);
extern int32_t       get_encoder_count(void);
extern void          set_encoder_inverted(bool inverted);
extern bool          get_encoder_inverted(void);
extern EncoderStatus increment_encoder_from_GPIO(bool a_high, bool b_high);
extern EncoderStatus get_encoder_status(void);

#endif //CANCODER_ENCODER_H
