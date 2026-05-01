/*
 * bno055.c
 *
 * Minimal BNO055 fused heading driver over I2C2.
 * Wiring: PE4 = I2C2SCL, PE5 = I2C2SDA, BNO055 address 0x28 by default.
 */

#include "bno055.h"
#include "Timer.h"
#include <inc/tm4c123gh6pm.h>
#include <stdint.h>

#define BNO055_REG_CHIP_ID            0x00
#define BNO055_REG_EUL_HEADING_LSB    0x1A
#define BNO055_REG_CALIB_STAT         0x35
#define BNO055_REG_UNIT_SEL           0x3B
#define BNO055_REG_OPR_MODE           0x3D
#define BNO055_REG_PWR_MODE           0x3E
#define BNO055_REG_SYS_TRIGGER        0x3F

#define BNO055_CHIP_ID                0xA0
#define BNO055_MODE_CONFIG            0x00
#define BNO055_MODE_NDOF              0x0C
#define BNO055_PWR_NORMAL             0x00
#define BNO055_UNIT_DEGREES           0x00

#define I2C_MCS_RUN                   0x01
#define I2C_MCS_START                 0x02
#define I2C_MCS_STOP                  0x04
#define I2C_MCS_ACK                   0x08
#define I2C_MCS_BUSY                  0x01
#define I2C_MCS_ERROR                 0x02

static int i2c_wait_done(void)
{
    int timeout = 50000;

    while ((I2C2_MCS_R & I2C_MCS_BUSY) && timeout > 0)
    {
        timeout--;
    }

    if (timeout == 0)
    {
        return 0;
    }

    return (I2C2_MCS_R & I2C_MCS_ERROR) == 0;
}

static void i2c2_init(void)
{
    SYSCTL_RCGCI2C_R |= 0x04;   // I2C2
    SYSCTL_RCGCGPIO_R |= 0x10;  // Port E

    while ((SYSCTL_PRI2C_R & 0x04) == 0) {}
    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {}

    GPIO_PORTE_AFSEL_R |= 0x30;     // PE4/PE5 alternate function
    GPIO_PORTE_DEN_R |= 0x30;
    GPIO_PORTE_AMSEL_R &= ~0x30;
    GPIO_PORTE_ODR_R |= 0x20;       // SDA open drain
    GPIO_PORTE_PCTL_R = (GPIO_PORTE_PCTL_R & ~0x00FF0000) | 0x00330000;

    I2C2_MCR_R = 0x10;              // master enable
    I2C2_MTPR_R = 7;                // 100 kHz from 16 MHz system clock
}

static int bno055_write8(uint8_t reg, uint8_t value)
{
    I2C2_MSA_R = (BNO055_I2C_ADDR << 1);
    I2C2_MDR_R = reg;
    I2C2_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    if (!i2c_wait_done())
    {
        I2C2_MCS_R = I2C_MCS_STOP;
        return 0;
    }

    I2C2_MDR_R = value;
    I2C2_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
    return i2c_wait_done();
}

static int bno055_read_len(uint8_t reg, uint8_t *data, uint8_t len)
{
    uint8_t i;

    if (len == 0)
    {
        return 0;
    }

    I2C2_MSA_R = (BNO055_I2C_ADDR << 1);
    I2C2_MDR_R = reg;
    I2C2_MCS_R = I2C_MCS_START | I2C_MCS_RUN;
    if (!i2c_wait_done())
    {
        I2C2_MCS_R = I2C_MCS_STOP;
        return 0;
    }

    I2C2_MSA_R = (BNO055_I2C_ADDR << 1) | 0x01;

    if (len == 1)
    {
        I2C2_MCS_R = I2C_MCS_START | I2C_MCS_STOP | I2C_MCS_RUN;
        if (!i2c_wait_done())
        {
            return 0;
        }
        data[0] = I2C2_MDR_R & 0xFF;
        return 1;
    }

    I2C2_MCS_R = I2C_MCS_START | I2C_MCS_RUN | I2C_MCS_ACK;
    if (!i2c_wait_done())
    {
        return 0;
    }
    data[0] = I2C2_MDR_R & 0xFF;

    for (i = 1; i < len - 1; i++)
    {
        I2C2_MCS_R = I2C_MCS_RUN | I2C_MCS_ACK;
        if (!i2c_wait_done())
        {
            return 0;
        }
        data[i] = I2C2_MDR_R & 0xFF;
    }

    I2C2_MCS_R = I2C_MCS_STOP | I2C_MCS_RUN;
    if (!i2c_wait_done())
    {
        return 0;
    }
    data[len - 1] = I2C2_MDR_R & 0xFF;

    return 1;
}

static int bno055_read8(uint8_t reg, uint8_t *value)
{
    return bno055_read_len(reg, value, 1);
}

int bno055_init(void)
{
    uint8_t chip_id = 0;
    int tries;

    i2c2_init();
    timer_waitMillis(700);

    for (tries = 0; tries < 10; tries++)
    {
        if (bno055_read8(BNO055_REG_CHIP_ID, &chip_id) && chip_id == BNO055_CHIP_ID)
        {
            break;
        }
        timer_waitMillis(100);
    }

    if (chip_id != BNO055_CHIP_ID)
    {
        return 0;
    }

    if (!bno055_write8(BNO055_REG_OPR_MODE, BNO055_MODE_CONFIG))
    {
        return 0;
    }
    timer_waitMillis(25);

    bno055_write8(BNO055_REG_PWR_MODE, BNO055_PWR_NORMAL);
    timer_waitMillis(10);
    bno055_write8(BNO055_REG_UNIT_SEL, BNO055_UNIT_DEGREES);
    bno055_write8(BNO055_REG_SYS_TRIGGER, 0x80); // use external crystal if present
    timer_waitMillis(10);

    if (!bno055_write8(BNO055_REG_OPR_MODE, BNO055_MODE_NDOF))
    {
        return 0;
    }
    timer_waitMillis(25);

    return 1;
}

float bno055_read_heading(void)
{
    uint8_t data[2];
    int16_t raw;

    if (!bno055_read_len(BNO055_REG_EUL_HEADING_LSB, data, 2))
    {
        return BNO055_HEADING_INVALID;
    }

    raw = (int16_t)((data[1] << 8) | data[0]);
    return bno055_normalize_angle((float)raw / 16.0f);
}

uint8_t bno055_read_calibration_status(void)
{
    uint8_t status = 0;

    if (!bno055_read8(BNO055_REG_CALIB_STAT, &status))
    {
        return 0;
    }

    return status;
}

int bno055_is_calibrated(void)
{
    uint8_t status = bno055_read_calibration_status();
    uint8_t sys = (status >> 6) & 0x03;
    uint8_t gyro = (status >> 4) & 0x03;
    uint8_t accel = (status >> 2) & 0x03;
    uint8_t mag = status & 0x03;

    return sys == 3 && gyro == 3 && accel == 3 && mag == 3;
}

float bno055_normalize_angle(float angle)
{
    while (angle >= 360.0f)
    {
        angle -= 360.0f;
    }

    while (angle < 0.0f)
    {
        angle += 360.0f;
    }

    return angle;
}

float bno055_angle_difference(float from_angle, float to_angle)
{
    float diff = bno055_normalize_angle(to_angle) - bno055_normalize_angle(from_angle);

    while (diff > 180.0f)
    {
        diff -= 360.0f;
    }

    while (diff < -180.0f)
    {
        diff += 360.0f;
    }

    return diff;
}
