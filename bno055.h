/*
 * bno055.h
 *
 * BNO055 fused orientation driver for TM4C123.
 * Default wiring uses I2C2 on PE4=SCL and PE5=SDA.
 */

#ifndef BNO055_H_
#define BNO055_H_

#include <stdint.h>

#define BNO055_I2C_ADDR_A             0x28
#define BNO055_I2C_ADDR_B             0x29
#define BNO055_I2C_ADDR               BNO055_I2C_ADDR_A

#define BNO055_HEADING_INVALID        (-1.0f)
#define BNO055_TURN_TOLERANCE_DEG     2.0f

int bno055_init(void);
float bno055_read_heading(void);
uint8_t bno055_read_calibration_status(void);
int bno055_is_calibrated(void);
float bno055_normalize_angle(float angle);
float bno055_angle_difference(float from_angle, float to_angle);

#endif /* BNO055_H_ */
