//
// Created by Benjamin Scholar on 1/27/20.
//

#include "eeprom.h"

#include "util.h"

#include "stm32f1xx_hal.h"
#include "main.h"

#include <stdlib.h>

#define FANCY_US_DELAY

#define I2C_SPEED 100
#define I2C_DELAY_US 4 // us

#define HIGH 1
#define LOW  0


// PRIVATE VARIABLES
static bool started = false;

// private function declarations
static void I2C_delay();
static void arbitration_lost();

static void set_SDA(bool high);
static void set_SCL(bool high);
static bool read_SDA();
static bool read_SCL();

// function definitions

#ifndef FANCY_US_DELAY
void I2C_delay() {
    __volatile int v;
    int i;
    for(i = 0; i < I2C_SPEED / 2; ++i) {
        v;
    }
}
#else
void I2C_delay() {
    DWT_Delay(I2C_DELAY_US);
}
#endif

void arbitration_lost() {
    // what do we want this to do
    // is this bad? probably
}

void set_SDA(bool high) {
    HAL_GPIO_WritePin(MEM_SDA_GPIO_Port, MEM_SDA_Pin, high);
}

bool read_SDA() {
    return HAL_GPIO_ReadPin(MEM_SDA_GPIO_Port, MEM_SDA_Pin);
}

void set_SCL(bool high) {
    HAL_GPIO_WritePin(MEM_SCL_GPIO_Port, MEM_SCL_Pin, high);
}

bool read_SCL() {
    return HAL_GPIO_ReadPin(MEM_SCL_GPIO_Port, MEM_SCL_Pin);
}


void i2c_start_condition() {
    if(started) {
        set_SDA(HIGH);
        I2C_delay();
        set_SCL(HIGH);
        while(!read_SCL()) {
            // some sort of delay
            // well try 2 us
            DWT_Delay(2);
        }
        I2C_delay();
    }
    if(!read_SDA()) {
        arbitration_lost();
    }
    set_SDA(LOW);
    I2C_delay();
    set_SCL(LOW);
    started = true;
}

void i2c_stop_condition() {
    set_SDA(LOW);
    I2C_delay();
    set_SCL(HIGH);
    while(!read_SCL()) {
        DWT_Delay(2);
    }
    I2C_delay();
    set_SDA(HIGH);
    I2C_delay();
    if(!read_SDA()) {
        arbitration_lost();
    }
    started = false;
}

void i2c_write_bit(bool bit) {
    if(bit) {
        set_SDA(HIGH);
    } else {
        set_SDA(LOW);
    }
    I2C_delay();
    set_SCL(HIGH);
    I2C_delay();
    while(!read_SCL()) {
        DWT_Delay(2);
    }

//    if(bit && !read_SDA()) {
//        arbitration_lost();
//    }

    set_SCL(LOW);
}


bool i2c_read_bit() {
    bool bit;
    set_SDA(HIGH);
    I2C_delay();
    set_SCL(HIGH);

    while(!read_SCL()) {
        DWT_Delay(2);
    }
    I2C_delay();
    bit = read_SDA();
    set_SCL(LOW);
    return bit;
}


bool i2c_write_byte(uint8_t byte, bool start, bool stop) {
    uint8_t bit;
    bool nack;

    if(start) i2c_start_condition();

    for(bit = 0; bit < 8;++bit) {
        i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    nack = i2c_read_bit();

    if(stop) i2c_stop_condition();

    return nack;
}

uint8_t i2c_read_byte(bool nack, bool stop) {
    uint8_t byte = 0;
    uint8_t bit;

    for(bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | i2c_read_bit();
    }
    i2c_write_bit(nack);

    if(stop) i2c_stop_condition();

    return byte;
}

bool i2c_send_byte(uint8_t address, uint8_t data) {
    // send control byte
    if(i2c_write_byte(WRITE_BYTE, true, false)) {
        // send address
        if(i2c_write_byte(address, false, false)) {
            // write data
            if(i2c_write_byte(data, false, true)) return true;
        }
    }
    return false;
}

void i2c_receive_byte(uint8_t address, uint8_t* ret) {
    // send control byte
    if(i2c_write_byte(READ_BYTE, true, false)) {
        // send address byte
        if(i2c_write_byte(address, false, false)) {
            // recieve
            *ret = i2c_read_byte(false, true);
            return;
        }
    }
    *ret = 0;
}

void i2c_receive_continuous_bytes(uint8_t address, uint8_t* buffer, size_t size) {
    if(i2c_write_byte(READ_BYTE, true, false)) {
        // send address byte
        if(i2c_write_byte(address, false, false)) {
            // read requested number of bytes
            size_t i;
            const size_t end = size - 1;
            for(i = 0; i < size; i++) {
                buffer[i] = i2c_read_byte((i != end), (i == end));
            }
        }
    }
}

