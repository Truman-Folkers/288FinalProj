/*
 * servo.c
 *
 *  Created on: Apr 9, 2026
 *      Author: ppetsche
 */

#include "Timer.h"
#include "lcd.h"
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "servo.h"

void servo_init(void)
{
    uint32_t period = 320000;
    uint32_t pulse_width = 24000;   // about 90 degrees
    uint32_t match_value = period - pulse_width;

    // enable clock to Timer1 and Port B
    SYSCTL_RCGCTIMER_R |= 0x02;
    SYSCTL_RCGCGPIO_R |= 0x02;

    // wait until peripherals are ready
    while ((SYSCTL_PRTIMER_R & 0x02) == 0) {}
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {}

    // configure PB5 for T1CCP1
    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_DIR_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_AMSEL_R &= ~0x20;
    GPIO_PORTB_PCTL_R = (GPIO_PORTB_PCTL_R & ~0x00F00000) | 0x00700000;

    // disable Timer1B before setup
    TIMER1_CTL_R &= ~0x00000100;

    // 16-bit timer configuration
    TIMER1_CFG_R = 0x4;

    // Timer1B periodic PWM mode, count-down
    TIMER1_TBMR_R = 0xA;

    // non-inverted output
    TIMER1_CTL_R &= ~0x00004000;

    // set 20 ms period
    TIMER1_TBPR_R = (period >> 16) & 0xFF;
    TIMER1_TBILR_R = period & 0xFFFF;

    // set initial pulse width
    TIMER1_TBPMR_R = (match_value >> 16) & 0xFF;
    TIMER1_TBMATCHR_R = match_value & 0xFFFF;

    // enable Timer1B
    TIMER1_CTL_R |= 0x00000100;
}

void servo_move(uint16_t degrees)
{
    uint32_t pulse_width;
    uint32_t period = 320000;
    uint32_t match_value;

    if (degrees > 180)
    {
        degrees = 180;
    }

    // 1 ms to 2 ms pulse over 0 to 180 degrees
    pulse_width = 16000 + ((16000 * degrees) / 180);

    match_value = period - pulse_width;

    TIMER1_TBPMR_R = (match_value >> 16) & 0xFF;
    TIMER1_TBMATCHR_R = match_value & 0xFFFF;

    timer_waitMillis(200);
}
