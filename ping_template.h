/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */
#ifndef PING_H_
#define PING_H_

#include <stdint.h>
#include <inc/tm4c123gh6pm.h>
#include <stdbool.h>
#include "driverlib/interrupt.h"


extern volatile uint32_t g_start_time;
extern volatile uint32_t g_end_time;
typedef volatile enum{LOW, HIGH, DONE} State;
volatile extern State g_state;

/**
 * Initialize ping sensor. Uses PB3 and Timer 3B
 */
void ping_init (void);

/**
 * @brief Trigger the ping sensor
 */
void ping_trigger (void);

/**
 * @brief Timer3B ping ISR
 */
void TIMER3B_Handler(void);

/**
 * @brief Calculate the distance in cm
 *
 * @return Distance in cm
 */
float ping_getDistance (void);

#endif /* PING_H_ */
