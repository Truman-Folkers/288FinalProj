//#include "open_interface.h"
//#include "movement.h"
//#include "cyBot_Scan.h"
//#include "uart-interrupt.h"
//#include "Timer.h"
//#include "lcd.h"
//#include <stdio.h>
//#include <math.h>
///**
// * main.c from Hannah/Parker
// * need to review movement.c
// */
//
//
//int main(void)
// {
//
//   //initialize everything else
//   timer_init();
//   lcd_init();
//   uart_interrupt_init();
//   oi_t *sensor_data = oi_alloc();
//   oi_init(sensor_data);
//
//   //initialize cyBOT
//   cyBOT_init_Scan(0b0111);
//   right_calibration_value = 306250;
//   left_calibration_value = 1319500;
//
//   //loop variables
//   int i;
//   int j;
//   int m;
//
//
//   //initializing scan
//   cyBOT_Scan_t scan;
//   cyBOT_Scan(0, &scan);
//
//
//   //list of objects with distance and angle
//   float object_distance[10];
//   int object_angle[10];
//
//   //array to map object coordinates
//   float main_map_x[10];
//   float main_map_y[10];
//
//
//
//
//   //variables to track stats of smallest object
//   float small_object_width = 1000.0;
//   int small_object_angle;
//   float small_object_distance;
//
//   //variables to track current object
//   int angle_start = -1;
//   int angle_end = -1;
//   int radial_width;
//   float lin_width;
//   int center_of_object;
//
//   //distance away from object
//   float distance;
//
//
//   //boolean, if 1, last scan saw an object
//   int distance_last = 0;
//
//   uart_sendStr("Angle(deg)\tDistance(m)\r\n");
//
//   //scans the field 0 to 180
//   for(i = 0; i <= 180; i+=2){
//
//      char data[20];
//      distance = 0;
//
//      //scans each object three times
//      for(j = 0; j<3; j+= 1){
//          cyBOT_Scan(i, &scan);
//          distance = distance + scan.IR_raw_val;
//      }
//
//      //IR conversion
//      distance = (500000.0 / (2*distance - 1070)) / 3.0;
//
//      //sends distance to PuTTY
//      sprintf(data, "%d\t%.1f\r\n",i,distance);
//      uart_sendStr(data);
//
//      //if object exists in IR range
//      if(distance < 51){
//
//          //if new object
//          if(!distance_last){
//              angle_start = i;
//              distance_last = 1;
//          }
//
//          //update end value
//          angle_end = i;
//      }
//
//      //no object in IR range
//      else{
//
//          //if the last scan had an object (end of object)
//          if(distance_last & (angle_start != angle_end)){
//
//              //updates last object values
//              radial_width = (angle_end - angle_start);
//              center_of_object = (angle_start + angle_end)/2;
//              cyBOT_Scan(center_of_object, &scan);
//              float ping_distance = scan.sound_dist;
//              lin_width =  2 * ping_distance * sin((radial_width * M_PI / 180.0)/2.0);
//
//              //sends object info to PuTTY
//              char data[100];
//              sprintf(data, "%d\t\t%d\t\t%d\t\t%.1f\r\n", object_count, center_of_object, (int)ping_distance, lin_width);
//              uart_sendStr(data);
//
//              //if last object was smallest thus far
//              if(lin_width < small_object_width){
//                  small_object_width = lin_width;
//                  small_object_angle = center_of_object;
//                  small_object_distance = ping_distance;
//                  smallest_object_num = object_count;
//              }
//
//              //updates object counter and arrays
//              object_angle[object_count] = center_of_object;
//              object_distance[object_count] = ping_distance;
//              object_count++;
//          }
//
//          distance_last = 0;
//
//          }
//       }
//
//
//   //make field map
//   for(m = 0; m < object_count; m++){
//       float angle2 = object_angle[m] * M_PI /180.0;
//       main_map_x[m] = object_distance[m] * cos(angle2) + robot_x;
//       main_map_y[m] = object_distance[m] * sin(angle2) + robot_y;
//   }
//
//
//   //copy map to movement
//   for(m = 0; m < object_count; m++){
//       map_x[m] = main_map_x[m];
//       map_y[m] = main_map_y[m];
//   }
//
//
//
//
//   //cyBOT turns toward smallest object (constants are offsets)
//   if(small_object_angle > 90){
//       turn_left(sensor_data, small_object_angle - 95);
//   }
//
//   else{
//       turn_right(sensor_data, 85 - small_object_angle);
//   }
//
//   //move to the smallest object
//   move_forward_two(sensor_data, (small_object_distance-8) *10);
//
//   oi_setWheels(0,0);
//
//   return 0;
//
//}
//
