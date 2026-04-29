///**
// *
// * sensor_test.c
// *
// * Tests sensors for hole and tape
// */
//
//#include "timer.h"
//#include "lcd.h"
//#include "open_interface.h"
//#include "sensor_test.h"
//#include <stdint.h>
//#include <inc/tm4c123gh6pm.h>
//
// void test_hole(oi_t *sensor_data){
//
//
//   //drives forward into hole. When cliff senses the hole, it backs up
//   //purpose: test the front cliff sensors
//    oi_setWheels(100,100);
//    while(!(sensor_data.cliffFrontLeft | sensor_data.cliffFrontRight)){};
//    oi_setWheels(-100,-100);
//
//    //test light bump sensors
//    //supposed to detect walls and low objects, like the hole
//    //packet id 45-51
//
// }
void test_hole(oi_t *sensor_data){


  //drives forward into hole. When cliff senses the hole, it backs up
  //purpose: test the front cliff sensors
   oi_setWheels(100,100);
   while(!(sensor_data.cliffFrontLeft | sensor_data.cliffFrontRight)){};
   oi_setWheels(-100,-100);

   //test light bump sensors
   //supposed to detect walls and low objects, like the hole
   //packet id 45-51

   while(1){
    oi_update(sensor_data);
   }
   if(lightBumpLeftSignal > 3500){
    sendString("COL:%i, %i", 150, lightBumpLeftSignal);
   }
   if(lightBumpFrontLeftSignal > 3500){
    sendString("COL:%i, %i", 125, lightBumpFrontLeftSignal);
   }
   if(lightBumpCenterLeftSignal > 3500){
    sendString("COL:%i, %i", 100, lightBumpCenterLeftSignal);
   }
   if(lightBumpCenterRightSignal > 3500){
    sendString("COL:%i, %i", 80, lightBumpCenterRightSignal);
   }
   if(lightBumpFrontRightSignal > 3500){
    sendString("COL:%i, %i", 55, lightBumpFrontRightSignal);
   }
   if(lightBumpRightSignal > 3500){
    sendString("COL:%i, %i", 30, lightBumpRightSignal);
   }
   

}