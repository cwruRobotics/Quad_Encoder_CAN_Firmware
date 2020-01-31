//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_CAN_API_H
#define CANCODER_CAN_API_H

#include "stm32f1xx_hal.h"

#include "types.h"
#include "encoder.h"

#include <stdio.h>
#include <stdbool.h>

// VARIABLES
extern uint8_t           CAN_id               ;
extern bool              CAN_connected        ;
extern HAL_StatusTypeDef CAN_status           ;

// USER CONFIGURABLE VARIABLES
extern uint16_t CAN_outgoing_message_period_ms;

// GENERAL API
extern void read_CAN_id_from_jumpers();


// INCOMING FRAME API
extern void process_incoming_frame(CAN_RxHeaderTypeDef*, uint8_t*);

extern void process_reset_ticks_request();
extern void process_set_ticks_request();
extern void process_set_polarity_request();
extern void process_set_feedback_period_request();


// OUTGOING FRAME API
extern bool CAN_should_update(uint32_t);
extern bool send_CAN_update(CAN_HandleTypeDef*, Frame*, uint8_t);

#endif //CANCODER_CAN_API_H
