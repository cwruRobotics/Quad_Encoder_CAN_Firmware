//
// Created by Benjamin Scholar on 1/27/20.
//

#include "CAN_api.h"

#include <string.h>

// Define externed variables
uint8_t           CAN_id                         = 0b00000000 ;
uint32_t          last_message_sent_ms           = 0          ;
bool              CAN_connected                  = false      ;
HAL_StatusTypeDef CAN_status                     = 0x00       ;
uint32_t          CAN_outgoing_message_period_ms = 0          ;

void process_incoming_frame(CAN_RxHeaderTypeDef* frame) {

}

bool CAN_should_update(uint32_t current_time_ms) {
    bool ret = (current_time_ms - last_message_sent_ms) > CAN_outgoing_message_period_ms;
    if(ret) last_message_sent_ms = current_time_ms;
    return ret;
}

bool send_CAN_update(CAN_HandleTypeDef* hcan, Frame* frame, uint8_t id) {
    static const uint8_t message_length = 6; // we could possibly change this as needed. But do we need to?
    // initialize stuff
    CAN_TxHeaderTypeDef hddr;
    uint32_t mailbox;
    uint8_t data[message_length];

    // set all data to 0
    memset(&data, 0, sizeof(data));

    // configure CAN message
    hddr.StdId = id;
    hddr.IDE = CAN_ID_STD;
    hddr.RTR = CAN_RTR_DATA;
    hddr.DLC = message_length;

    // set data frame desired frame
    data[0] = frame->type;
    data[1] = frame->error_type;
    memcpy(data + 2, &(frame->ticks), 4); // should we still send this if there is an error?

    // send the message and check for error
    return HAL_CAN_AddTxMessage(hcan, &hddr, data, &mailbox) == HAL_OK;
}