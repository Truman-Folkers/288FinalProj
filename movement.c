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
#include "bno055.h"
#include "movement.h"
#include "string.h"
#include <stdio.h>
#include <math.h>

#define LIGHT_BUMP_OBJECT_THRESHOLD 2800
#define SENSOR_REPORT_TICKS 10
#define TELEMETRY_REPORT_TICKS 5
#define MOVEMENT_LOOP_MS 20

volatile float map_x[10];
volatile float map_y[10];
volatile float robot_x = 0;
volatile float robot_y = 0;
volatile float robot_angle = 0;

volatile int smallest_object_num = 0;
volatile int object_count = 0;
volatile movement_cmd_t current_cmd = CMD_STOP;
volatile float imu_target_heading = 0.0f;
volatile int bumpLeftSensed = 0;
volatile int bumpRightSensed = 0;

// Function to send a string to PuTTY one character at a time
void sendMovement(int scale, char type )
{
    char sentString[16];
    sprintf(sentString, "\r\nMOV:%d%c\r\n", scale, type);
    int i = 0;
    while (sentString[i] != '\0')
    {
        uart_sendChar(sentString[i]);
        i++;
    }
}

void sendBump(int l, int r)
{
    char sentString[20];
    sprintf(sentString, "\r\nBUMP:%d%d\r\n", l, r);

    int i = 0;
    while (sentString[i] != '\0')
    {
        uart_sendChar(sentString[i]);
        i++;
    }
}

void sendTelemetry(void)
{
    char sentString[48];
    float heading = get_robot_heading_imu();

    if (heading != BNO055_HEADING_INVALID)
    {
        robot_angle = heading * M_PI / 180.0f;
    }
    else
    {
        heading = robot_angle * 180.0f / M_PI;
    }

    sprintf(sentString, "\r\nPOS:%.2f,%.2f,%.2f\r\n", robot_x, robot_y, heading);

    int i = 0;
    while (sentString[i] != '\0')
    {
        uart_sendChar(sentString[i]);
        i++;
    }
}

void sendCol(int sensor_id, int value){
    char sentString[24];
    sprintf(sentString, "\r\nCOL:%d,%d\r\n", sensor_id, value);

    int i = 0;
    while (sentString[i] != '\0')
    {
        uart_sendChar(sentString[i]);
        i++;
    }
}

static void report_adc_object_sensors(oi_t *sensor_data)
{
    static unsigned int lastMask = 0;
    static int tick = 0;
    unsigned int mask = 0;
    int report = 0;

    struct sensor_report
    {
        int id;
        int value;
        int active;
    } sensors[] = {
        {4,  sensor_data->lightBumpLeftSignal,        sensor_data->lightBumpLeftSignal        > LIGHT_BUMP_OBJECT_THRESHOLD},
        {2,  sensor_data->lightBumpFrontLeftSignal,   sensor_data->lightBumpFrontLeftSignal   > LIGHT_BUMP_OBJECT_THRESHOLD},
        {1,  sensor_data->lightBumpCenterLeftSignal,  sensor_data->lightBumpCenterLeftSignal  > LIGHT_BUMP_OBJECT_THRESHOLD},
        {6,  sensor_data->lightBumpCenterRightSignal, sensor_data->lightBumpCenterRightSignal > LIGHT_BUMP_OBJECT_THRESHOLD},
        {3,  sensor_data->lightBumpFrontRightSignal,  sensor_data->lightBumpFrontRightSignal  > LIGHT_BUMP_OBJECT_THRESHOLD},
        {5,  sensor_data->lightBumpRightSignal,       sensor_data->lightBumpRightSignal       > LIGHT_BUMP_OBJECT_THRESHOLD},
        {7,  sensor_data->cliffLeftSignal,            sensor_data->cliffLeft},
        {8,  sensor_data->cliffFrontLeftSignal,       sensor_data->cliffFrontLeft},
        {9,  sensor_data->cliffFrontRightSignal,      sensor_data->cliffFrontRight},
        {10, sensor_data->cliffRightSignal,           sensor_data->cliffRight},
    };

    int i;
    for (i = 0; i < (int)(sizeof(sensors) / sizeof(sensors[0])); i++)
    {
        if (sensors[i].active)
        {
            mask |= (1u << i);
        }
    }

    tick++;
    report = (mask != lastMask) || (mask != 0 && tick >= SENSOR_REPORT_TICKS);
    if (!report)
    {
        return;
    }

    tick = 0;
    lastMask = mask;

    for (i = 0; i < (int)(sizeof(sensors) / sizeof(sensors[0])); i++)
    {
        if (sensors[i].active)
        {
            sendCol(sensors[i].id, sensors[i].value);
        }
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

float get_robot_heading_imu(void)
{
    return bno055_read_heading();
}

float turn_to_heading_imu(oi_t *sensor_data, float target_heading)
{
    float current_heading;
    float diff;
    int elapsed_ms = 0;

    target_heading = bno055_normalize_angle(target_heading);
    current_heading = get_robot_heading_imu();

    if (current_heading == BNO055_HEADING_INVALID)
    {
        oi_setWheels(0, 0);
        return BNO055_HEADING_INVALID;
    }

    while (elapsed_ms < IMU_TURN_TIMEOUT_MS)
    {
        int speed;

        oi_update(sensor_data);
        report_adc_object_sensors(sensor_data);

        if (sensor_data->bumpLeft || sensor_data->bumpRight)
        {
            oi_setWheels(0, 0);
            current_cmd = CMD_STOP;
            sendBump(sensor_data->bumpLeft, sensor_data->bumpRight);
            sendTelemetry();
            return get_robot_heading_imu();
        }

        current_heading = get_robot_heading_imu();
        if (current_heading == BNO055_HEADING_INVALID)
        {
            oi_setWheels(0, 0);
            return BNO055_HEADING_INVALID;
        }

        diff = bno055_angle_difference(current_heading, target_heading);

        if (fabs(diff) <= IMU_TURN_TOLERANCE_DEG)
        {
            oi_setWheels(0, 0);
            robot_angle = current_heading * M_PI / 180.0f;
            sendTelemetry();
            return current_heading;
        }

        speed = (fabs(diff) < 15.0f) ? 80 : 145;

        if (diff > 0.0f)
        {
            oi_setWheels(speed, -speed);
        }
        else
        {
            oi_setWheels(-speed, speed);
        }

        timer_waitMillis(IMU_TURN_LOOP_MS);
        elapsed_ms += IMU_TURN_LOOP_MS;
    }

    oi_setWheels(0, 0);
    sendTelemetry();
    return get_robot_heading_imu();
}

float turn_left_imu(oi_t *sensor_data, float degrees)
{
    float start_heading = get_robot_heading_imu();

    if (start_heading == BNO055_HEADING_INVALID)
    {
        oi_setWheels(0, 0);
        return BNO055_HEADING_INVALID;
    }

    return turn_to_heading_imu(sensor_data, start_heading + degrees);
}

float turn_right_imu(oi_t *sensor_data, float degrees)
{
    float start_heading = get_robot_heading_imu();

    if (start_heading == BNO055_HEADING_INVALID)
    {
        oi_setWheels(0, 0);
        return BNO055_HEADING_INVALID;
    }

    return turn_to_heading_imu(sensor_data, start_heading - degrees);
}

double turn_left(oi_t *sensor_data, double degrees)
{
    turn_left_imu(sensor_data, (float)degrees);
    return degrees;
}

double turn_right(oi_t *sensor_data, double degrees)
{
    turn_right_imu(sensor_data, (float)degrees);
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
    static int bump_latched = 0;
    static int telemetry_tick = 0;

    float distance1;
    float heading;

    oi_update(sensor_data);
    report_adc_object_sensors(sensor_data);

    distance1 = sensor_data->distance;

    heading = get_robot_heading_imu();
    if (heading != BNO055_HEADING_INVALID)
    {
        robot_angle = heading * M_PI / 180.0f;
    }

    // Encoder distance updates position; IMU heading owns orientation.
    robot_x += (distance1 / 10.0f) * cos(robot_angle);
    robot_y += (distance1 / 10.0f) * sin(robot_angle);

    while (robot_angle > M_PI)
        robot_angle -= 2 * M_PI;
    while (robot_angle < -M_PI)
        robot_angle += 2 * M_PI;

    int bumpLeftNow = sensor_data->bumpLeft;
    int bumpRightNow = sensor_data->bumpRight;
    int bumpNow = bumpLeftNow || bumpRightNow;

    // New bump detected
    if (bumpNow && !bump_latched)
    {
        oi_setWheels(0, 0);
        current_cmd = CMD_STOP;
        sendBump(bumpLeftNow, bumpRightNow);
        bump_latched = 1;
    }

    // Bump released
    else if (!bumpNow && bump_latched)
    {
        sendBump(0, 0);
        bump_latched = 0;
    }

    // While bump is still pressed, ONLY block forward
    if (bumpNow && current_cmd == CMD_FORWARD)
    {
        current_cmd = CMD_STOP;
    }

//    if(sensor_data->lightBumpLeftSignal > 2800){
//        sendCol(6, sensor_data->lightBumpLeftSignal);
//    }
//    if(sensor_data->lightBumpFrontLeftSignal > 2800){
//            sendCol(5, sensor_data->lightBumpFrontLeftSignal);
//        }
//    if(sensor_data->lightBumpCenterLeftSignal > 2800){
//            sendCol(4, sensor_data->lightBumpCenterLeftSignal);
//        }
//    if(sensor_data->lightBumpCenterRightSignal > 2800){
//            sendCol(3, sensor_data->lightBumpCenterRightSignal);
//        }
//    if(sensor_data->lightBumpFrontRightSignal > 2800){
//            sendCol(2, sensor_data->lightBumpFrontRightSignal);
//        }
//    if(sensor_data->lightBumpRightSignal > 2800){
//            sendCol(6, sensor_data->lightBumpRightSignal);
//        }


    switch (current_cmd)
    {
    case CMD_FORWARD:
        oi_setWheels(150, 150);
        break;

    case CMD_BACKWARD:
        oi_setWheels(-150, -150);
        break;

    case CMD_LEFT:
        oi_setWheels(150, -150);
        break;

    case CMD_RIGHT:
        oi_setWheels(-150, 150);
        break;

    case CMD_RIGHT_90:
        turn_right_imu(sensor_data, 90.0f);
        current_cmd = CMD_STOP;
        break;

    case CMD_LEFT_90:
        turn_left_imu(sensor_data, 90.0f);
        current_cmd = CMD_STOP;
        break;

    case CMD_TURN_TO_HEADING:
        turn_to_heading_imu(sensor_data, imu_target_heading);
        current_cmd = CMD_STOP;
        break;

    case CMD_STOP:
    default:
        oi_setWheels(0, 0);
        break;
    }

    telemetry_tick++;
    if (telemetry_tick >= TELEMETRY_REPORT_TICKS || current_cmd == CMD_STOP)
    {
        telemetry_tick = 0;
        sendTelemetry();
    }

    timer_waitMillis(MOVEMENT_LOOP_MS);
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
