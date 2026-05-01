/*
 * movement.h
 *
 *  Created on: Feb 5, 2026
 *      Author: ppetsche
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include "open_interface.h"
//#include "cyBot_Scan.h"

#define IMU_TURN_TOLERANCE_DEG 2.0f
#define IMU_TURN_TIMEOUT_MS    6000
#define IMU_TURN_LOOP_MS       10

extern volatile float map_x[10];
extern volatile float map_y[10];
extern volatile float robot_x;
extern volatile float robot_y;
extern volatile float robot_angle;
extern volatile int object_count;
extern volatile int smallest_object_num;

typedef enum
{
   CMD_STOP,
   CMD_FORWARD,
   CMD_BACKWARD,
   CMD_LEFT,
   CMD_RIGHT,
   CMD_RIGHT_90,
   CMD_LEFT_90,
   CMD_TURN_TO_HEADING
} movement_cmd_t;

extern volatile movement_cmd_t current_cmd;
extern volatile float imu_target_heading;

double move_forward(oi_t *sensor_data, double distance_mm);
double turn_left(oi_t *sensor_data, double degrees);
double turn_right(oi_t *sensor_data, double degrees);
float turn_left_imu(oi_t *sensor_data, float degrees);
float turn_right_imu(oi_t *sensor_data, float degrees);
float turn_to_heading_imu(oi_t *sensor_data, float target_heading);
float get_robot_heading_imu(void);
double move_backward(oi_t *sensor_data, double distance_mm);
double move_forward_two(oi_t *sensor_data, double distance_moved);
void avoidObjects(oi_t *sensor_data);
void movement_update(oi_t *sensor_data);
void sendTelemetry(void);

#endif /* MOVEMENT_H_ */
