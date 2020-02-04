//
// Created by Benjamin Scholar on 1/27/20.
//

#include "eeprom.h"

#include "stm32f1xx_hal.h"
#include "main.h"

#define I2C_SPEED 100
#define I2C_DELAY 1 //ms

// VARIABLES
static bool started = false;


void I2C_delay() {
    __volatile int v;
    int i;
    for(i = 0; i < I2C_SPEED / 2; ++i) {
        v;
    }
}

void arbitration_lost() {
    // what do we want this to do
}

void set_SDA_low() {
    HAL_GPIO_WritePin(MEM_SDA_GPIO_Port, MEM_SDA_Pin, GPIO_PIN_RESET);
}

void set_SDA_high() {
    HAL_GPIO_WritePin(MEM_SDA_GPIO_Port, MEM_SDA_Pin, GPIO_PIN_SET);
}

bool read_SDA() {
    return false;
}

void set_SCL_low() {
    HAL_GPIO_WritePin(MEM_SCL_GPIO_Port, MEM_SCL_Pin, GPIO_PIN_RESET);
}

void set_SCL_high() {
    HAL_GPIO_WritePin(MEM_SCL_GPIO_Port, MEM_SCL_Pin, GPIO_PIN_SET);
}

bool read_SCL() {
    return false;
}


void i2c_start_cond() {
    if(started) {
        set_SDA_high();
        I2C_delay();
        set_SCL_high();
        while(!read_SCL()) {
            // some sort of delay
            // well try 1 ms for now
            HAL_Delay(1);
        }
        I2C_delay();
    }
    if(!read_SDA()) {
        arbitration_lost();
    }
    set_SDA_low();
    I2C_delay();
    set_SCL_low();
    started = true;
}

void i2c_stop_cond() {
    set_SDA_low();
    I2C_delay();
    set_SCL_high();

    while(!read_SCL()) {
        HAL_Delay(1);
    }
    I2C_delay();
    set_SDA_high();
    I2C_delay();
    if(!read_SDA()) {
        arbitration_lost();
    }
    started = false;
}

void i2c_write_bit(bool bit) {
    if(bit) {
        set_SDA_high();
    } else {
        set_SDA_low();
    }
    I2C_delay();
    set_SCL_high();
    I2C_delay();
    while(!read_SCL()) {
        HAL_Delay(1);
    }

    if(bit && !read_SDA()) {
        arbitration_lost();
    }

    set_SCL_low();
}


bool i2c_read_bit() {
    bool bit;
    set_SDA_high();
    I2C_delay();
    set_SCL_high();

    while(!read_SCL()) {
        HAL_Delay(1);
    }
    I2C_delay();
    bit = read_SDA();
    set_SCL_low();
    return bit;
}


bool i2c_write_byte(bool send_start, bool send_stop, unsigned char byte) {
    unsigned bit;
    bool nack;

    if(send_start) {
        i2c_start_cond();
    }
    for(bit = 0; bit < 8;++bit) {
        i2c_write_bit((byte & 0x80) != 0);
        byte <<= 1;
    }
    nack = i2c_read_bit();
    if(send_stop) {
        i2c_stop_cond();
    }
    return nack;
}

unsigned char i2c_read_byte(bool nack, bool send_stop) {
    unsigned char byte = 0;
    unsigned char bit;

    for(bit = 0; bit < 8; ++bit) {
        byte = (byte << 1) | i2c_read_bit();
    }
    i2c_write_bit(nack);
    if(send_stop) {
        i2c_stop_cond();
    }
    return byte;
}


