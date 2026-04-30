/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping_template.h"
#include "Timer.h"
#include "lcd.h"
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>


// Global shared variables
// Use extern declarations in the header file

volatile uint32_t g_start_time = 0;
volatile uint32_t g_end_time = 0;
volatile State g_state = LOW; // State of ping echo pulse

void ping_init (void){

    //turn on timer 3 and port B
    SYSCTL_RCGCTIMER_R |= 0x08;
    SYSCTL_RCGCGPIO_R |= 0x02;

    //busy-wait for system clocks
    while((SYSCTL_PRTIMER_R & 0x08) == 0) {}
    while((SYSCTL_PRGPIO_R & 0x02) == 0) {}


    //set PB3 to alt
    GPIO_PORTB_DEN_R |= 0x08;
    GPIO_PORTB_AFSEL_R |= 0x08;
    GPIO_PORTB_PCTL_R &= ~0x0000F000;
    GPIO_PORTB_PCTL_R |= 0x00007000;

    //disables timer
    TIMER3_CTL_R &= ~0x100;

    //sets timer to 16-bit
    TIMER3_CFG_R = 0x4;

    //sets timer to count-down, capture, edge-time mode
    TIMER3_TBMR_R &= ~0x10;
    TIMER3_TBMR_R |= 0x7;

    //defines an event as both positive and negative edge
    TIMER3_CTL_R |= 0xC00;

    //sets start clock time to 24-bit
    TIMER3_TBPR_R = 0xFF;
    TIMER3_TBILR_R = 0xFFFF;

    //clears interrupt on bit 10 of timer interrupts
    TIMER3_ICR_R |= 0x400;

    //enables capture mode event interrupt
    TIMER3_IMR_R |= 0x400;

    //sets interrupt priority of 3B to 1
    NVIC_PRI9_R = (NVIC_PRI9_R & 0xFFFFFF0F) | (0x00000020);

    //enables interrupt 36 (TIMER 3B)
    NVIC_EN1_R |= 0x10;

    IntRegister(INT_TIMER3B, TIMER3B_Handler); //

    IntMasterEnable(); //

    // Configure and enable the timer
    TIMER3_CTL_R |= 0x100; //
}

void ping_trigger (void){

    g_state = LOW;
    g_start_time = 0;
    g_end_time = 0;

    // Disable timer and disable timer interrupt
    TIMER3_CTL_R &= ~0x100;
    TIMER3_IMR_R &= ~0x400;

    // Disable alternate function (disconnect timer from port pin)
    GPIO_PORTB_AFSEL_R &= ~0x08;

    //set PB3 to output
    GPIO_PORTB_DIR_R |= 0x08;

    //write 0,1,0 to PB3 to trigger start pulse
    GPIO_PORTB_DATA_R &= ~0x08;
    timer_waitMicros(2);

    GPIO_PORTB_DATA_R |= 0X08;
    timer_waitMicros(10);

    GPIO_PORTB_DATA_R &= ~0x08;

    // Re-enable alternate function, timer interrupt, and timer
    TIMER3_ICR_R |= 0x400;
    GPIO_PORTB_DIR_R &= ~0x08;
    GPIO_PORTB_AFSEL_R |= 0x08;
    TIMER3_IMR_R |= 0x400;
    TIMER3_CTL_R |= 0x100;

}

void TIMER3B_Handler(void){

    //check interrupt source
    if(TIMER3_MIS_R & 0x400){


    //clear interrupt
    TIMER3_ICR_R |= 0x400;

    //if positive edge
    if(g_state == LOW){
        //set state to high, set start time to current clock
        g_start_time = TIMER3_TBR_R;
        g_state = HIGH;
    }

    else if(g_state == HIGH){
        //set end time to current clock, state to done (runs main)
        g_end_time = TIMER3_TBR_R;
        g_state = DONE;
    }
    }

}

float ping_getDistance (void){

    uint32_t pulse_width;

    //if overflow hasn't occurred
    if(g_start_time >= g_end_time){
        pulse_width = g_start_time - g_end_time;
    }

    //if overflow has occurred
    else{
        pulse_width = g_start_time + (0xFFFFFF - g_end_time);
    }

    //convert pulse from cycles to seconds
    float time = pulse_width * (1.0/16000000.0);

    //distance in cm
    float distance = (34300 * time) / 2.0;

    //reset state
    g_state = LOW;

    return distance;


}


