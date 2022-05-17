#ifndef SINGLE_RANGING__H
#define SINGLE_RANGING__H

#include "vl53l0x_api.h"
#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"

VL53L0X_Error measure(VL53L0X_Dev_t *pMyDevice, uint8_t count);
VL53L0X_Error initDevice(VL53L0X_Dev_t *pMyDevice);

#endif

