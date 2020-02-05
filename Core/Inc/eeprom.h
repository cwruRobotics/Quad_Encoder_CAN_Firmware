//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_EEPROM_H
#define CANCODER_EEPROM_H

#include <stdbool.h>
#include <inttypes.h>
#include <sched.h>


// references
// https://calcium3000.wordpress.com/2016/08/19/i2c-bit-banging-tutorial-part-i/
// http://ww1.microchip.com/downloads/en/DeviceDoc/24AA0-24LC01B-24FC01-Data-Sheet-20001711L.pdf

// constants
#define MIN_MEMORY_LOCATION 0   // bytes
#define MAX_MEMORY_LOCATION 127 // bytes

// defined in the data sheet
#define WRITE_BYTE 0b10101110
#define READ_BYTE  0b10101111

// memory addresses
#define ENCODER_TICKS_LOCATION    0x00 // 4 bytes wide
#define ENCODER_POLARITY_LOCATION 0x04 // 1 bytes wide
#define FEEDBACK_PERIOD_LOCATION  0x05 // 2 bytes wide

// public function definitions
extern void i2c_start_condition();
extern void i2c_stop_condition();

// read and write functions
extern bool i2c_send_byte(uint8_t address, uint8_t data);
extern void i2c_receive_byte(uint8_t address, uint8_t* ret);
extern void i2c_receive_continuous_bytes(uint8_t address, uint8_t* buffer, size_t size);

// probably shouldn't use these directly
extern void i2c_write_bit(bool bit);
extern bool i2c_read_bit();
extern bool i2c_write_byte(uint8_t byte, bool start, bool stop);
extern uint8_t i2c_read_byte(bool nack, bool stop);

#endif //CANCODER_EEPROM_H
