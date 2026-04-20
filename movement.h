/*
 * movement.h
 *
 *  Created on: Feb 5, 2026
 *      Author: ppetsche
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"
#include "movement.h"
#include "cyBot_Scan.h"


   extern volatile float map_x[10];
   extern volatile float map_y[10];
   extern volatile float robot_x;
   extern volatile float robot_y;
   extern volatile int object_count;
   extern volatile int smallest_object_num;


double move_forward(oi_t*sensor_data, double distance_mm);
double turn_left(oi_t*sensor_data, double degrees);
double turn_right(oi_t*sensor_data, double degrees);
double move_backward(oi_t*sensor_data, double distance_mm);
double move_forward_two(oi_t* sensor_data, double distance_moved);
void avoidObjects(oi_t*sensor_data);


#endif /* MOVEMENT_H_ */
