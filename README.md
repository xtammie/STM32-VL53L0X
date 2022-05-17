The code in this repository uses the ST-library for the VL53L0X Time of Flight sensor.
The code in this repository is written for the STM32F0_Discovery.
The code in this repository uses the Standard Peripheral Library for the STM32F0.
The code in this repository uses USART to print.
To alter the code for your own microcontroller, you need to alter the helper.c for USART.
To alter the code for your own microcontroller, you need to alter the VL53L0X_write_multi(), the VL53L0X_read_multi() and the VL53L0X_set_gpio() in the vl53l0x_i2c_platform.c for I2C.
To alter the code for your own microcontroller, you need to delete device specific files and includes.
