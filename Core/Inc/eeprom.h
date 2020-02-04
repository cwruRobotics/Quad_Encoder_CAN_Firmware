//
// Created by Benjamin Scholar on 1/27/20.
//

#ifndef CANCODER_EEPROM_H
#define CANCODER_EEPROM_H

#include <stdbool.h>

extern void I2C_delay();
extern void arbitration_lost();

extern void set_SDA_high();
extern void set_SDA_low();
extern void set_SCL_high();
extern void set_SCL_low();
extern bool read_SDA();
extern bool read_SCL();

extern void i2c_start_cond();
extern void i2c_stop_cond();
extern void i2c_write_bit(bool bit);
extern bool i2c_read_bit();
extern bool i2c_write_byte(bool send_start, bool send_stop, unsigned char byte);
extern unsigned char i2c_read_byte(bool nack, bool send_stop);

#endif //CANCODER_EEPROM_H
