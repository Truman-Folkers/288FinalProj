/*
 * movement.c
 *
 *  Created on: Feb 5, 2026
 *      Author: ppetsche
 */

//#include "cyBot_Scan.h"
#include "Timer.h"
#include "open_interface.h"
#include "uart-interrupt.h"
#include "movement.h"
#include "string.h"
#include <math.h>

volatile float map_x[10];
volatile float map_y[10];
volatile float robot_x = 0;
volatile float robot_y = 0;
volatile float robot_angle = 0;

volatile int smallest_object_num = 0;
volatile int object_count = 0;
volatile movement_cmd_t current_cmd = CMD_STOP;
volatile int bumpLeftSensed = 0;
volatile int bumpRightSensed = 0;

// Function to send a string to PuTTY one character at a time
void sendMovement(int scale, char type )
{
    char sentString[16];
    sprintf(sentString, "\nMOV:%d%c", scale, type);
    int i = 0;
    while (sentString[i] != '\0')
    {
        uart_sendChar(sentString[i]);
        i++;
    }
}

void sendBump(int l, int r){
    char sentString[16];
    sprintf(sentString, "\nBUMP:%d%d", l, r);
    int i = 0;
    while (sentString[i] != '\0')
    {
        uart_sendChar(sentString[i]);
        i++;
    }
}

double move_forward(oi_t *sensor_data, double distance_mm)
{

    double distance_moved = 0;

    oi_setWheels(200, 200);

    while (distance_moved < distance_mm)
    {
        oi_update(sensor_data);
        distance_moved = distance_moved + sensor_data->distance;
    }

    oi_setWheels(0, 0);
    return distance_moved;
}

double move_backward(oi_t *sensor_data, double distance_mm)
{

    double distance_moved = 0;

    oi_setWheels(-200, -200);

    while (distance_moved > distance_mm)
    {
        oi_update(sensor_data);
        distance_moved = distance_moved + sensor_data->distance;
    }

    sendMovement(((int)(distance_mm * 10)), 'b');
    oi_setWheels(0, 0);
    return distance_moved;
}

double turn_left(oi_t *sensor_data, double degrees)
{
    double angle_robot = 0;

    oi_setWheels(150, -150);

    while (angle_robot < degrees - 8) //degrees - 8 for CyBot 24
    {
        oi_update(sensor_data);
        angle_robot = angle_robot + sensor_data->angle;
    }

    sendMovement((int)fabs(angle_robot), 'l');
    oi_setWheels(0, 0);
    return degrees;
}

double turn_right(oi_t *sensor_data, double degrees)
{
    double two = 0 - degrees;
    double angle_robot = 0;

    oi_setWheels(-150, 150);

    while (angle_robot > two + 10) //two + 10 for CyBot 24
    {
        oi_update(sensor_data);
        angle_robot = angle_robot + sensor_data->angle;
    }
    sendMovement((int)fabs(angle_robot), 'r');
    oi_setWheels(0, 0);
    return degrees;
}

float object_distance[10];
int object_angle[10];
int object_width[10];



double move_forward_two(oi_t *sensor_data, double distance_moved)
{
    double sum = 0;
    int i;
    float distance1;
    float angle1;

    while (sum < distance_moved)
    {

        oi_setWheels(200, 200);
        oi_update(sensor_data);

        distance1 = sensor_data->distance;
        angle1 = sensor_data->angle * M_PI / 180.0;

        // update robot position on map
        robot_x += distance1 * cos(robot_angle);
        robot_y += distance1 * sin(robot_angle);
        robot_angle += angle1;

        sum += distance1;

        if (sensor_data->bumpLeft)
        {
            turn_right(sensor_data, 90);
            move_forward(sensor_data, 100);
            turn_left(sensor_data, 90);
            move_forward(sensor_data, 160);
            turn_left(sensor_data, 90);
            move_forward(sensor_data, 100);
            turn_right(sensor_data, 90);
            sum += 160;
        }

        else if (sensor_data->bumpRight)
        {
            turn_left(sensor_data, 90);
            move_forward(sensor_data, 100);
            turn_right(sensor_data, 90);
            move_forward(sensor_data, 160);
            turn_right(sensor_data, 90);
            move_forward(sensor_data, 100);
            turn_left(sensor_data, 90);
            sum += 160;
        }

        for (i = 0; i < object_count; i++)
        {
            if (i == smallest_object_num)
            {
                continue;
            }

            float object_x = map_x[i] - robot_x;
            float object_y = map_y[i] - robot_y;

            float distance2 = sqrt(object_x * object_x + object_y * object_y);

            if (distance2 < 15)
            {

                float angle = atan2(object_y, object_x) - robot_angle;

                while (angle > M_PI)
                    angle -= 2 * M_PI;
                while (angle < -M_PI)
                    angle += 2 * M_PI;

                if (fabs(angle) < (20 * M_PI / 180.0))
                {

                    oi_setWheels(0, 0);
                    avoidObjects(sensor_data);
                    break;
                }
            }
        }
    }

    oi_setWheels(0, 0);
    return sum;
}

void avoidObjects(oi_t *sensor_data)
{

    oi_setWheels(0, 0);

    turn_left(sensor_data, 45);
    move_forward(sensor_data, 70);

    turn_right(sensor_data, 45);
}

void movement_update(oi_t *sensor_data)
{
    float distance1;
    float angle1;
    bumpLeftSensed = 0;
    bumpRightSensed = 0;
    oi_update(sensor_data);

    distance1 = sensor_data->distance;
    angle1 = sensor_data->angle * M_PI / 180.0;

    // Update robot position on map
    robot_x += distance1 * cos(robot_angle);
    robot_y += distance1 * sin(robot_angle);
    robot_angle += angle1;

    // Keep robot_angle bounded
    while (robot_angle > M_PI)
        robot_angle -= 2 * M_PI;
    while (robot_angle < -M_PI)
        robot_angle += 2 * M_PI;

    if(sensor_data->bumpLeft && bumpLeftSensed == 0){
        current_cmd = CMD_STOP;
        bumpLeftSensed = 1;
    }
    if(sensor_data->bumpRight && bumpRightSensed == 0){
        current_cmd = CMD_STOP;
        bumpRightSensed = 1;
    }
    if(bumpRightSensed || bumpLeftSensed){
        sendBump(bumpLeftSensed, bumpRightSensed);
    }



    /*
     * Report this tick's deltas straight to the GUI.  No accumulation, no
     * thresholds — Python handles whatever stream we send.  We do skip
     * deltas that are below 1 (mm/deg) so we don't spam "MOV:0f" lines.
     *
     * We might have to fine tune it if the GUI is getting to many 0 updates that accumulate to to much.
     *
     * Sign convention: + distance = forward, - distance = backward;
     *                  + angle    = left    (CCW),  - angle    = right.
     */
    int d_mm  = (int)sensor_data->distance;
    int a_deg = (int)sensor_data->angle;

    if (d_mm > 0)        sendMovement( d_mm, 'f');
    else if (d_mm < 0)   sendMovement(-d_mm, 'b');

    if (a_deg > 0)       sendMovement( a_deg, 'l');
    else if (a_deg < 0)  sendMovement(-a_deg, 'r');

    switch (current_cmd)
    {
    case CMD_FORWARD:
        oi_setWheels(150, 150);
//        sendString(((int)(distance_mm * 10)), 'f');
        break;

    case CMD_BACKWARD:
        oi_setWheels(-150, -150);
//        sendString(((int)(distance_mm * 10)), 'b');
        break;

    case CMD_LEFT:
        oi_setWheels(150, -150);
        break;

    case CMD_RIGHT:
        oi_setWheels(-150, 150);
        break;

    case CMD_RIGHT_90:
        turn_right(sensor_data, 90);
        current_cmd = CMD_STOP;
        break;

    case CMD_LEFT_90:
        turn_left(sensor_data, 90);
        current_cmd = CMD_STOP;
        break;

    case CMD_STOP:
    default:
        oi_setWheels(0, 0);
        break;
    }
}


//    // Handle bump sensors
//    if (sensor_data->bumpLeft)
//    {
//        oi_setWheels(0, 0);
//
//        turn_right(sensor_data, 90);
//        move_forward(sensor_data, 100);
//        turn_left(sensor_data, 90);
//        move_forward(sensor_data, 160);
//        turn_left(sensor_data, 90);
//        move_forward(sensor_data, 100);
//        turn_right(sensor_data, 90);
//
//        current_cmd = CMD_STOP;
//        return;
//    }
//    else if (sensor_data->bumpRight)
//    {
//        oi_setWheels(0, 0);
//
//        turn_left(sensor_data, 90);
//        move_forward(sensor_data, 100);
//        turn_right(sensor_data, 90);
//        move_forward(sensor_data, 160);
//        turn_right(sensor_data, 90);
//        move_forward(sensor_data, 100);
//        turn_left(sensor_data, 90);
//
//        current_cmd = CMD_STOP;
//        return;
//    }
//    int i;
//    // Check mapped objects
//    for (i = 0; i < object_count; i++)
//    {
//        if (i == smallest_object_num)
//        {
//            continue;
//        }
//
//        float object_x = map_x[i] - robot_x;
//        float object_y = map_y[i] - robot_y;
//
//        float distance2 = sqrt(object_x * object_x + object_y * object_y);
//
//        if (distance2 < 15)
//        {
//            float angle = atan2(object_y, object_x) - robot_angle;
//
//            while (angle > M_PI)
//                angle -= 2 * M_PI;
//            while (angle < -M_PI)
//                angle += 2 * M_PI;
//
//            if (fabs(angle) < (20 * M_PI / 180.0))
//            {
//                oi_setWheels(0, 0);
//                avoidObjects(sensor_data);
//                current_cmd = CMD_STOP;
//                return;
//            }
//        }
//    }
