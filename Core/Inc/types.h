//
// Created by Benjamin Scholar on 1/22/20.
//

#ifndef CANCODER_TYPES_H
#define CANCODER_TYPES_H

#include <stdbool.h>

typedef enum {
  FRAME_TYPE_TICKS    = 0x00,
  FRAME_TYPE_VELOCITY = 0x01, // we may not use this for now
  FRAME_TYPE_ERROR    = 0xFF
} FrameType;

typedef enum {
  ERROR_TYPE_NONE = 0x00
} ErrorType;

typedef struct {
  uint8_t type;
  uint8_t error_type;
  uint32_t ticks;
} Frame;

// enum specifying commands that can be read from incoming CAN messages.
typedef enum {
  REQUEST_NONE                = 0x00,
  REQUEST_RESET_TICKS         = 0x01,
  REQUEST_SET_TICKS           = 0x02,
  REQUEST_SET_POLARITY        = 0x03,
  REQUEST_SET_FEEDBACK_PERIOD = 0x04,
} IncomingRequestType;

typedef enum {
    ENCODER_OK           = 0x00,
    ENCODER_COUNT_ERROR  = 0x01
} EncoderStatus;

#endif //CANCODER_TYPES_H
