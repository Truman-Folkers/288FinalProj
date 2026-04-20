/*
 * servo.c
 *
 *  Created on: Apr 9, 2026
 *      Author: ppetsche
 */

#include "Timer.h"
#include "lcd.h"
#include "driverlib/interrupt.h"
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include "servo.h"

void servo_init(void){

    //init timer 1, port B
    SYSCTL_RCGCTIMER_R |= 0x02;
    SYSCTL_RCGCGPIO_R |= 0x02;

    //busy-waits for system clocks
    while((SYSCTL_RCGCTIMER_R & 0x02) == 0) {};
    while((SYSCTL_RCGCGPIO_R & 0x02) == 0) {};


    //enable alt function on PB5
    GPIO_PORTB_DEN_R |= 0x20;
    GPIO_PORTB_AFSEL_R |= 0x20;
    GPIO_PORTB_PCTL_R &= ~0x00F00000;
    GPIO_PORTB_PCTL_R |= 0x00700000;

    //disables timer before initializing
    TIMER1_CTL_R &= ~0x00000100;

    //Sets timer config to 16-bit
    TIMER1_CFG_R = 0x04;

    //sets timer mode to PWM, edge-count, periodic
    TIMER1_TBMR_R &= ~0x00000014;
    TIMER1_TBMR_R |= 0xA;


    //Keeps output as-is (recommended by datasheet)
    TIMER1_CTL_R &= ~0x00004000;


    //full clock cycle for PWM (20 ms)
    uint32_t period = 320000;


    //set clock value to period (2 ms)
    TIMER1_TBPR_R = (period >> 16) & 0xFF;
    TIMER1_TBILR_R = (period & 0xFFFF);

    //enables timer
    TIMER1_CTL_R |= 0x00000100;


}


void servo_move(uint16_t degrees){

    //convert degrees to pulse width (clock cycles)
    uint32_t pulse_width = 16000 + ((16000 * degrees) / 180);

    uint32_t period = 320000;

    //how long to stay HIGH
    uint32_t match_value = period - pulse_width;

    //upper 8 bits are stored
    TIMER1_TBPMR_R = (match_value >> 16) & 0xFF;

    //lower 16 bits stored
    TIMER1_TBMATCHR_R = (match_value & 0xFFFF);


    //delay so that the servo can move
    timer_waitMillis(200); //might need to be dependent on the degrees

}
