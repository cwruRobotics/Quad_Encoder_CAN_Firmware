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
const static uint8_t CAN_message_length = 6;

extern uint8_t           CAN_id               ;
extern uint32_t          last_message_sent_ms ;
extern bool              CAN_connected        ;
extern HAL_StatusTypeDef CAN_status           ;

// USER CONFIGURABLE VARIABLES
extern uint32_t CAN_outgoing_message_period_ms;

// GENERAL API


// INCOMING FRAME API
void process_incoming_frame(CAN_RxHeaderTypeDef* frame);

void process_reset_ticks_request();
void process_set_ticks_request();
void process_set_polarity_request();
void process_set_feedback_period_request();


// OUTGOING FRAME API
bool CAN_should_update(uint32_t current_time_ms);
bool send_CAN_update(CAN_HandleTypeDef* hcan, Frame* frame, uint8_t id);

#endif //CANCODER_CAN_API_H
