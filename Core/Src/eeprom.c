//
// Created by Benjamin Scholar on 1/27/20.
//

#include "eeprom.h"

#include "stm32f1xx_hal.h"



void I2C_delay() {
    __volatile int v;
    int i;
    for(i = 0; i < I2C_SPEED / 2; ++i) {
        v;
    }
}

//bool read_SDA() {
//    return HAL_GPIO_ReadPin();
//}