/*
 * servo.h
 *
 *  Created on: Apr 9, 2026
 *      Author: ppetsche
 */

#ifndef SERVO_H_
#define SERVO_H_


#include "Timer.h"
#include "lcd.h"
#include "driverlib/interrupt.h"
#include <stdint.h>
#include <inc/tm4c123gh6pm.h>


void servo_init(void);
void servo_move(uint16_t degrees);




#endif /* SERVO_H_ */
