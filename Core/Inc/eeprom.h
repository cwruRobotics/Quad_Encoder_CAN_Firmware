//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_EEPROM_H
#define CANCODER_EEPROM_H

#include <stdbool.h>

#define I2C_SPEED 100

extern bool started;

void I2C_delay();
void set_SDA_high();
void set_SDA_low();
void set_SCL_high();
void set_SCL_low();
bool read_SDA();
bool read_SCL();

#endif //CANCODER_EEPROM_H
