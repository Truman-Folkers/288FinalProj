/*
 * movement.c
 *
 *  Created on: Feb 5, 2026
 *      Author: ppetsche
 */

#include "cyBot_Scan.h"
#include "Timer.h"
#include "open_interface.h"
#include "movement.h"
#include "string.h"
#include <math.h>

   volatile float map_x[10];
   volatile float map_y[10];
   volatile float robot_x = 0;
   volatile float robot_y = 0;

   volatile int smallest_object_num = 0;
   volatile int object_count = 0;




double move_forward(oi_t*sensor_data, double distance_mm){

    double distance_moved = 0;

    oi_setWheels(200,200);

    while(distance_moved < distance_mm){
        oi_update(sensor_data);
        distance_moved = distance_moved + sensor_data -> distance;

    }

    oi_setWheels(0,0);
    return distance_moved;
}


double move_backward(oi_t*sensor_data, double distance_mm){

    double distance_moved = 0;

    oi_setWheels(-200,-200);

    while(distance_moved > distance_mm){
        oi_update(sensor_data);
        distance_moved = distance_moved + sensor_data -> distance;

    }

    oi_setWheels(0,0);
    return distance_moved;
}




double turn_left(oi_t*sensor_data, double degrees){
    double angle_robot = 0;

    oi_setWheels(200,-200);

    while(angle_robot < degrees){
        oi_update(sensor_data);
        angle_robot = angle_robot + sensor_data -> angle;
    }
    oi_setWheels(0,0);
    return degrees;

}


double turn_right(oi_t*sensor_data, double degrees){
    double two = 0-degrees;
    double angle_robot = 0;

    oi_setWheels(-200,200);

    while(angle_robot > two){
        oi_update(sensor_data);
        angle_robot = angle_robot + sensor_data -> angle;
    }
    oi_setWheels(0,0);
    return degrees;

}




float object_distance[10];
int object_angle[10];
int object_width[10];



float robot_angle = 0;



double move_forward_two(oi_t* sensor_data, double distance_moved){
    double sum = 0;
    int i;
    float distance1;
    float angle1;


        while(sum < distance_moved){

            oi_setWheels(200,200);
            oi_update(sensor_data);

            distance1 = sensor_data -> distance;
            angle1 = sensor_data -> angle * M_PI /180.0;

            //update robot position on map
            robot_x += distance1 * cos(robot_angle);
            robot_y += distance1 * sin(robot_angle);
            robot_angle += angle1;


            sum += distance1;


            if(sensor_data -> bumpLeft){
                turn_right(sensor_data, 90);
                move_forward(sensor_data, 100);
                                turn_left(sensor_data, 90);
                                move_forward(sensor_data,160);
                                turn_left(sensor_data, 90);
                                move_forward(sensor_data, 100);
                                turn_right(sensor_data, 90);
                                sum+=160;

            }

            else if(sensor_data -> bumpRight){
                turn_left(sensor_data, 90);
                move_forward(sensor_data, 100);
                turn_right(sensor_data, 90);
                move_forward(sensor_data, 160);
                turn_right(sensor_data, 90);
                move_forward(sensor_data, 100);
                turn_left(sensor_data, 90);
                sum+=160;

            }


            for(i = 0; i < object_count; i++){
                if(i == smallest_object_num){
                    continue;
                }

                float object_x = map_x[i] - robot_x;
                float object_y = map_y[i] - robot_y;

                float distance2 = sqrt(object_x*object_x + object_y*object_y);

                if(distance2 < 15){

                    float angle = atan2(object_y, object_x) - robot_angle;

                    while(angle > M_PI) angle -= 2*M_PI;
                    while(angle < -M_PI) angle += 2*M_PI;

                    if(fabs(angle) < (20 * M_PI /180.0)){

                        oi_setWheels(0,0);
                        avoidObjects(sensor_data);
                        break;

                    }

                }



            }

        }

        oi_setWheels(0,0);
        return sum;
}


void avoidObjects(oi_t*sensor_data){

    oi_setWheels(0,0);

    turn_left(sensor_data, 45);
    move_forward(sensor_data, 70);

    turn_right(sensor_data, 45);

}

