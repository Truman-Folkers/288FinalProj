

/**
 * main.c
 */
int main(void)
{
    //start
	return 0;
}




//truman and andres lab 7 :

///**
// * main.c
// */
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
////#include "cyBot_uart.h"
//#include "cyBot_Scan.h"
//#include "lcd.h"
//#include "uart-interrupt.h"
//#include <math.h>
//#include "movement.h"
//#include "open_interface.h"
//#include "adc.h"
//#include "PINGLogAnalyzer.h"
//
//
//// Function to send a string to PuTTY one character at a time
//void sendString(char *str) {
//    int i = 0;
//    while (str[i] != '\0') {
//        uart_sendChar(str[i]);
//        i++;
//    }
//}
//typedef struct {
//    float cm;
//    float adc;
//} ir_point_t;
//
//static const ir_point_t ir_table[] = {
//    {10.0, 2491.00},
//    {12.0, 2226.33},
//    {15.0, 1942.00},
//    {18.0, 1750.33},
//    {20.0, 1629.67},
//    {22.0, 1561.33},
//    {25.0, 1424.00},
//    {30.0, 1295.67},
//    {33.0, 1246.33},
//    {35.0, 1204.33},
//    {40.0, 1125.00},
//    {42.0, 1068.33},
//    {45.0, 1042.00},
//    {50.0, 984.33},
//    {55.0, 929.00},
//    {60.0, 886.33}
//};
//
//#define IR_TABLE_SIZE (sizeof(ir_table) / sizeof(ir_table[0]))
//float adc_to_cm(uint16_t adc_value) {
//    int i;
//
//    /* clamp above nearest calibrated range */
//    if (adc_value >= ir_table[0].adc) {
//        return ir_table[0].cm;
//    }
//
//    /* clamp below farthest calibrated range */
//    if (adc_value <= ir_table[IR_TABLE_SIZE - 1].adc) {
//        return ir_table[IR_TABLE_SIZE - 1].cm;
//    }
//
//    /* find interval where adc_value lies between two table points */
//    for (i = 0; i < IR_TABLE_SIZE - 1; i++) {
//        float adc_high = ir_table[i].adc;
//        float adc_low  = ir_table[i + 1].adc;
//
//        if (adc_value <= adc_high && adc_value >= adc_low) {
//            float cm_high = ir_table[i].cm;
//            float cm_low  = ir_table[i + 1].cm;
//
//            /* linear interpolation */
//            return cm_high + ((adc_value - adc_high) * (cm_low - cm_high)) / (adc_low - adc_high);
//        }
//    }
//
//    /* fallback, should not happen */
//    return -1.0;
//}
//
//int main(void)
//{
////    char log_data[] = "0 35.93 1 35.86 2 35.87 3 35.83 4 35.82 5 35.43 6 35.34 7 35.35 8 35.29 9 34.83 10 35.22 11 35.22 12 34.79 13 34.79 14 34.77 15 34.78 16 34.80 17 34.79 18 34.74 19 34.75 20 34.74 21 34.74 22 34.76 23 34.80 24 34.80 25 34.80 26 34.81 27 34.81 28 34.80 29 34.80 30 34.83 31 34.84 32 35.27 33 35.28 34 35.32 35 35.33 36 35.40 37 35.39 38 35.43 39 35.86 40 35.91 41 35.92 42 36.39 43 36.44 44 37.31 45 37.49 46 37.48 47 39.63 48 111.66 49 112.15 50 112.24 51 111.77 52 112.24 53 112.44 54 113.19 55 112.38 56 113.23 57 113.73 58 113.67 59 114.21 60 267.25 61 261.40 62 261.31 63 266.45 64 262.35 65 260.32 66 282.21 67 282.15 68 280.94 69 281.60 70 266.18 71 280.54 72 284.05 73 31.13 74 29.37 75 28.93 76 28.47 77 28.43 78 28.37 79 28.37 80 28.32 81 28.31 82 28.28 83 28.27 84 28.23 85 28.22 86 28.24 87 28.24 88 28.20 89 28.20 90 28.19 91 28.13 92 28.13 93 28.17 94 27.73 95 27.71 96 27.71 97 27.70 98 27.71 99 27.69 100 28.12 101 28.13 102 28.14 103 28.14 104 28.13 105 28.18 106 28.18 107 28.18 108 28.23 109 28.22 110 28.23 111 28.27 112 28.28 113 28.28 114 28.32 115 28.38 116 28.37 117 28.43 118 28.37 119 29.36 120 29.35 121 30.52 122 38.87 123 38.86 124 39.25 125 38.76 126 37.88 127 37.83 128 37.84 129 37.79 130 37.33 131 37.34 132 37.28 133 37.28 134 37.28 135 37.23 136 36.81 137 36.81 138 36.78 139 36.76 140 36.76 141 36.76 142 36.74 143 36.77 144 36.76 145 36.77 146 36.76 147 36.76 148 36.76 149 36.81 150 36.83 151 36.80 152 36.80 153 36.86 154 37.29 155 37.27 156 37.33 157 37.32 158 37.32 159 37.81 160 37.83 161 38.32 162 37.90 163 38.39 164 38.37 165 39.27 166 40.19 167 46.16 168 47.03 169 47.47 170 224.87 171 224.91 172 224.88 173 225.35 174 225.69 175 225.77 176 224.47 177 225.71 178 226.22 179 226.63 180 225.75";
////
////    ObjectAnalyzer(log_data);
//    //=======================sending sensor info to putty (part b)=====================
//        char receivedChar;
//            char buffer[100];
//            cyBOT_Scan_t scanData;
//            oi_t *sensor_data = oi_alloc(); //allocating memory for the data
//
//
//
//            lcd_init();
//            timer_init();
//            uart_interrupt_init();
//            oi_init(sensor_data);
//            adc_init();
//
//
//            cyBOT_init_Scan(0b0111);
//
//             //Calibrate Servo
//            //cyBOT_SERVO_cal();
////            cyBOT_SERVO_cal();
//            right_calibration_value = 290500;
//            left_calibration_value = 1282750;
//
//            lcd_printf("Send 'm'");
//
//
//            cyBOT_Scan_t sensorVals[5];
//            int irPrev = 0;
//            int irAverage;
//            int startingAngle = 0;
//            int endingAngle = 0;
//            int inObject = 0;
//            int midAngle = 0;
//            int numObj = 0;
//            int inObjectCounter = 0;
//            int peakIR = 0;
//            int peakAngle = 0;
//            int riseCount = 0;
//            int angleTurnRate = 1;
//            int fallCount = 0;
//            int candidateStartAngle = 0;
//            struct object {
//                  int number;
//                  int angle;
//                  float distance_cm;
//                  int widthAng;
//                  float widthLin;
//                  int peakIR;
//              };
//            struct object objArr[100];
//
//            while(1) {
//
//                receivedChar = returnChar;
//
//                if(receivedChar == 'm') {
//
//                    lcd_clear();
//                    numObj = 0;
//                    irPrev = 0;
//                    inObject = 0;
//                    inObjectCounter = 0;
//                    startingAngle = 0;
//                    endingAngle = 0;
//                    peakIR = 0;
//                    peakAngle = 0;
//                    memset(objArr, 0, sizeof(objArr));
//                    lcd_printf("Scanning...");
//
//                    sendString("\r\nStarting Scan...\r\n");
//                    sendString("Angle\tPING(cm)\tIR Raw\r\n");
//                    sendString("--------------------------------\r\n");
//
//                    int angle;
//
//                    for(angle = 0; angle <= 180; angle += angleTurnRate) {
//
//                        cyBOT_Scan(angle, &scanData);
//                        sensorVals[0] = scanData;
//
//                        cyBOT_Scan(angle, &scanData);
//                        sensorVals[1] = scanData;
//
//                        cyBOT_Scan(angle, &scanData);
//                        sensorVals[2] = scanData;
//
//                        irAverage = (sensorVals[0].IR_raw_val + sensorVals[1].IR_raw_val + sensorVals[2].IR_raw_val) / 3;
//;
//
//
//                        //------START OBJ---------------
////                        if(irAverage - irPrev > 40 && angle > 1 && irAverage > 50 && inObject == 0){
////                            inObjectCounter++;
////                            if(inObjectCounter > 3){
////                                inObject = 1;
////                                startingAngle = angle;
////                                inObjectCounter = 0;
////                            }
////                            peakIR = irAverage;
////                        }
//
//
//                        // ---- OBJECT CONTINUE ----
////                       if(inObject && irAverage > peakIR) {
////                           peakIR = irAverage;
////                           peakAngle = angle;
////                       }
//
//                       //-----------END OBJ---------------
////                       if(irAverage - irPrev < -40 && angle > 1 && irAverage > 50 && inObject == 1){ //maybe not > 200
////                           endingAngle = angle;
////
////                           peakIR = 0;
////                           midAngle = (endingAngle + startingAngle) /2;
////
////                           objArr[numObj].angle = midAngle;
//////                           objArr[numObj].distance_cm = peakIR;
////                           objArr[numObj].number = numObj++;
////                           objArr[numObj].widthAng = endingAngle - startingAngle;
////
////                           endingAngle = 0;
////                           startingAngle = 0;
////                           inObject = 0;
////                       }
//                        //------chat logic-----
//                        if (inObject == 0) {
//                            // Looking for object start
//                            if ((irAverage - irPrev) > 40 && irAverage > 50) {
//                                if (riseCount == 0) {
//                                    candidateStartAngle = angle;   // save first rising edge
//                                }
//
//                                riseCount++;
//
//                                // require several rising samples before confirming object
//                                if (riseCount >= 3) {
//                                    inObject = 1;
//                                    startingAngle = candidateStartAngle;
//                                    riseCount = 0;
//                                    fallCount = 0;
//                                    peakIR = irAverage;
//                                    peakAngle = angle;
//                                }
//                            } else {
//                                // rise broke, so reset start detection
//                                riseCount = 0;
//                            }
//                        } else {
//                            // Already inside an object
//
//                            // track peak while inside object
//                            if (irAverage > peakIR) {
//                                peakIR = irAverage;
//                                peakAngle = angle;
//                            }
//
//                            // look for several falling samples before ending object
//                            if ((irAverage - irPrev) < -40 || irAverage <= 50) {
//                                fallCount++;
//
//                                if (fallCount >= 2) {
//                                    endingAngle = angle;
//
//                                    objArr[numObj].number = numObj;
//                                    objArr[numObj].angle = (startingAngle + endingAngle) / 2;   // or peakAngle
//                                    objArr[numObj].widthAng = endingAngle - startingAngle;
//                                    objArr[numObj].peakIR = peakIR;
//
//                                    numObj++;
//
//                                    inObject = 0;
//                                    fallCount = 0;
//                                    riseCount = 0;
//                                    peakIR = 0;
//                                    peakAngle = 0;
//                                }
//                            } else {
//                                // still inside object normally
//                                fallCount = 0;
//                            }
//                        }
//
//                        irPrev = irAverage;
//
//
////width logic?
//
//
//
//
////                        sprintf(buffer, "%d\t%.2f\t\t%d\t\t%d\t%d\r\n",
////                                angle,
////                                scanData.sound_dist,
////                                irAverage,
////                                inObject,
////                                midAngle);
//
//                       sprintf(buffer, "%d,%.2f,%d\n", angle, scanData.sound_dist, irAverage);
//
//                        sendString(buffer);
//
//                        timer_waitMillis(50);
//
//                        receivedChar = returnChar;
//                        if(receivedChar == 's'){
//                            break;
//                        }
//                    }
//
//
//                    //------------180 obj logic--------
//                    if (inObject == 1) {
//                        endingAngle = angle;
//
//                        objArr[numObj].number = numObj;
//                        objArr[numObj].angle = (startingAngle + endingAngle) / 2;
//                        objArr[numObj].widthAng = endingAngle - startingAngle;
//
//                        numObj++;
//
//                        inObject = 0;
//                    }
//
//
//                    //second scan for PING dist
//                    int i;
//                    for(i = 0; i < numObj; i++){
//                        cyBOT_Scan(objArr[i].angle, &scanData);
//                        timer_waitMillis(500);
//                        uint16_t adc = adc_read();
//                        float dist = adc_to_cm(adc);
//                        objArr[i].distance_cm = dist;
//                    }
//
//
//                    int j;
//                    int b;
//                    b = 1;
//                    struct object smallestObj = objArr[0];
//                    // ---- Print Object Data ----
//                    for(j = 0; j < numObj; j++) {
//                        objArr[j].widthLin = 2 * objArr[j].distance_cm * tan((objArr[j].widthAng * 3.14159268 / 180.0) / 2.0);
//                        if(objArr[j].widthAng < 5) {continue;}
//                        if(objArr[j].widthLin > 100 || objArr[j].widthLin < 0){continue;}
//                        if(objArr[j].angle > 180){continue;}
//                        if(objArr[j].distance_cm > 1000 || objArr[j].distance_cm < 0){continue;}
//                        if(objArr[j].widthLin < smallestObj.widthLin){smallestObj = objArr[j];}
//
//
//                        sprintf(buffer,
//                            "\r\nObject %d\r\n"
//                            "Center Angle: %d deg\r\n"
//                            "Distance: %.2f cm\r\n"
//                            "Width: %.2f cm\r\n"
//                            ,
//                            b++,
//                            objArr[j].angle,
//                            objArr[j].distance_cm,
//                            objArr[j].widthLin);
//
//
//                        sendString(buffer);
//
//                    }
//
//                    //--------move to smallest width------------
//                    int turnAng = 90 - smallestObj.angle;
//                    if(turnAng > 0){
//                        turn_right(sensor_data, turnAng);
//                    }else{
//                        turn_left(sensor_data, -turnAng);
//                    }
//                    if(smallestObj.distance_cm > 15.0){
//                        go(sensor_data, (smallestObj.distance_cm - 10) * 10);
//                        angleTurnRate = 3;
//                    }else{
//                        break;
//                    }
//
//                    sendString("\r\nScan Complete\r\n");
//                    lcd_clear();
//                    lcd_printf("Done");
//
//                }
//            }
//            oi_free(sensor_data); //free memory
//
//}

