//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_EEPROM_H
#define CANCODER_EEPROM_H

#define I2C_SPEED 100


#include <stdbool.h>

extern void I2C_delay();
extern void set_SDA_high();
extern void set_SDA_low();
extern void set_SCL_high();
extern void set_SCL_low();
extern bool read_SDA();
extern bool read_SCL();

#endif //CANCODER_EEPROM_H
