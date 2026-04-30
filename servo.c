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

//default for cal values, changed in main
//defined as pulse width for 0 and 180, respectively
volatile uint32_t right_cal_value;
volatile uint32_t left_cal_value;

void servo_init(void)
{
    uint32_t period = 320000;
    //uint32_t pulse_width = 24000;   // about 90 degrees
    //uint32_t match_value = period - pulse_width;

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

    uint32_t initial_pulse_width = left_cal_value;
    uint32_t initial_match_value = period - initial_pulse_width;

    TIMER1_TBPMR_R = (initial_match_value >> 16) & 0xFF;
    TIMER1_TBMATCHR_R = initial_match_value & 0xFFFF;

    // enable Timer1B
    TIMER1_CTL_R |= 0x00000100;
}

void servo_move(uint16_t degrees)
{
    uint32_t pulse_width;
    uint32_t period = 320000;
    uint32_t match_value;


    //NEW! uses cal values from 0 and 180 pulse width
    //creates a linear model for other pulse widths
    pulse_width = right_cal_value + (((left_cal_value - right_cal_value) * degrees) / 180);

    match_value = period - pulse_width;

    TIMER1_TBPMR_R = (match_value >> 16) & 0xFF;
    TIMER1_TBMATCHR_R = match_value & 0xFFFF;

    timer_waitMillis(50);
}


//NEW, TO CALIBRATE SERVO (must do after servo init)
void servo_calibrate(){

    //Turn on Port E timer
    SYSCTL_RCGCGPIO_R |= 0x10;
    while ((SYSCTL_PRGPIO_R & 0x10) == 0) {};

    //Enable digital function for buttons and make them inputs
    GPIO_PORTE_DIR_R &= 0xF0;
    GPIO_PORTE_DEN_R |= 0x0F;

    //start cal process
    int angle = 90;
    servo_move(angle);
    lcd_printf("Finding right...");

    //while button 4 isn't being pushed
    while((GPIO_PORTE_DATA_R & 0x08) != 0){

        //if button 2 is pushed, move right
        if((GPIO_PORTE_DATA_R & 0x02) == 0){
            angle -= 3;
            servo_move(angle);
        }

        //if button 1 is pushed, move left
        else if((GPIO_PORTE_DATA_R & 0x01) == 0){
            angle += 3;
            servo_move(angle);
        }
    }

    //sets right calibration value, prints to lcd
    int right = (TIMER1_TBPMR_R << 16) + TIMER1_TBMATCHR_R;
    lcd_printf("Right: %i", 320000-right);



    //waits until button 3 is pressed to continue
    while((GPIO_PORTE_DATA_R & 0x04) != 0){};

    lcd_printf("Finding left...");

    //while button 4 isn't being pushed
    while((GPIO_PORTE_DATA_R & 0x08) != 0){

        //if button 2 is pushed, move right
        if((GPIO_PORTE_DATA_R & 0x02) == 0){
            angle -= 3;
            servo_move(angle);
        }

        //if button 1 is pushed, move left
        else if((GPIO_PORTE_DATA_R & 0x01) == 0){
            angle += 3;
            servo_move(angle);
        }
    }

    //sets left calibration value, prints to lcd
    int left = (TIMER1_TBPMR_R << 16) + TIMER1_TBMATCHR_R;
//    left_cal_value = 320000 - left;
//    right_cal_value = 320000 - right;
    lcd_printf("Left: %i", 320000 - left);
    timer_waitMillis(2000);
}
