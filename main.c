// truman and andres lab 7 :

/**
 * main.c
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
// #include "cyBot_uart.h"
#include "servo.h" //cybotScan.h
#include "lcd.h"
#include "uart-interrupt.h"
#include <math.h>
#include "movement.h"
#include "open_interface.h"
#include "adc.h"
#include "ping_template.h"


// variable initialization
//  Object containment
struct object
{
    int number;
    int angle;
    float distance_cm;
    int widthAng;
    float widthLin;
    int peakIR;
};

//cyBOT_Scan_t sensorVals[5];
int irPrev = 0;
int irAverage;
int startingAngle = 0;
int endingAngle = 0;
int inObject = 0;
int midAngle = 0;
int numObj = 0;
int inObjectCounter = 0;
int peakIR = 0;
int peakAngle = 0;
int riseCount = 0;
int angleTurnRate = 2;
int fallCount = 0;
int candidateStartAngle = 0;
int scanStartAngle = 0;
int scanEndAngle = 180;

struct object objArr[10];
char buffer[128];

//cyBOT_Scan_t scanData;
char receivedChar;
char lastChar;
int m;

/*============================================= */

// Function to send a string to PuTTY one character at a time
void sendString(const char *str)
{
    int i = 0;
    while (str[i] != '\0')
    {
        uart_sendChar(str[i]);
        i++;
    }
}

static float read_ping_cm(void)
{
    int timeout = 0;

    ping_trigger();
    while (g_state != DONE && timeout < 400000)
    {
        timeout++;
    }

    if (g_state != DONE)
    {
        g_state = LOW;
        return 0.0;
    }

    return ping_getDistance();
}
typedef struct
{
    float cm;
    float adc;
} ir_point_t;

static const ir_point_t ir_table[] = {
    {10.0, 2491.00},
    {12.0, 2226.33},
    {15.0, 1942.00},
    {18.0, 1750.33},
    {20.0, 1629.67},
    {22.0, 1561.33},
    {25.0, 1424.00},
    {30.0, 1295.67},
    {33.0, 1246.33},
    {35.0, 1204.33},
    {40.0, 1125.00},
    {42.0, 1068.33},
    {45.0, 1042.00},
    {50.0, 984.33},
    {55.0, 929.00},
    {60.0, 886.33}};

#define IR_TABLE_SIZE (sizeof(ir_table) / sizeof(ir_table[0]))
float adc_to_cm(uint16_t adc_value)
{
    int i;

    /* clamp above nearest calibrated range */
    if (adc_value >= ir_table[0].adc)
    {
        return ir_table[0].cm;
    }

    /* clamp below farthest calibrated range */
    if (adc_value <= ir_table[IR_TABLE_SIZE - 1].adc)
    {
        return ir_table[IR_TABLE_SIZE - 1].cm;
    }

    /* find interval where adc_value lies between two table points */
    for (i = 0; i < IR_TABLE_SIZE - 1; i++)
    {
        float adc_high = ir_table[i].adc;
        float adc_low = ir_table[i + 1].adc;

        if (adc_value <= adc_high && adc_value >= adc_low)
        {
            float cm_high = ir_table[i].cm;
            float cm_low = ir_table[i + 1].cm;

            /* linear interpolation */
            return cm_high + ((adc_value - adc_high) * (cm_low - cm_high)) / (adc_low - adc_high);
        }
    }

    /* fallback, should not happen */
    return -1.0;
}

void scan_and_map(oi_t *sensor_data)
{
    lcd_clear();


    numObj = 0;
    irPrev = 0;
    inObject = 0;
    inObjectCounter = 0;
    startingAngle = 0;
    endingAngle = 0;
    peakIR = 0;
    peakAngle = 0;
    int k = 0;
    memset(objArr, 0, sizeof(objArr));
    lcd_printf("Scanning...");

    sendString("\r\nStarting Scan...\r\n");
    sendString("Angle\tPING(cm)\tIR Raw\r\n");
    sendString("--------------------------------\r\n");

    int angle;

    for (angle = scanStartAngle; angle <= scanEndAngle; angle += angleTurnRate)
    {

        int irSum = 0;
        float pingDist = 0.0;

        servo_move(angle);
        timer_waitMillis(100);

        pingDist = read_ping_cm();

        for (k = 0; k < 3; k++)
        {
            irSum += adc_read();
            timer_waitMillis(5);
        }
        irAverage = irSum / 3;

        //------Object detection logic-----
        if (inObject == 0)
        {
            // Looking for object start
            if ((irAverage - irPrev) > 40 && irAverage > 50)
            {
                if (riseCount == 0)
                {
                    candidateStartAngle = angle; // save first rising edge
                }

                riseCount++;

                // require several rising samples before confirming object
                if (riseCount >= 3)
                {
                    inObject = 1;
                    startingAngle = candidateStartAngle;
                    riseCount = 0;
                    fallCount = 0;
                    peakIR = irAverage;
                    peakAngle = angle;
                }
            }
            else
            {
                // rise broke, so reset start detection
                riseCount = 0;
            }
        }
        else
        {
            // Already inside an object

            // track peak while inside object
            if (irAverage > peakIR)
            {
                peakIR = irAverage;
                peakAngle = angle;
            }

            // look for several falling samples before ending object
            if ((irAverage - irPrev) < -40 || irAverage <= 50)
            {
                fallCount++;

                if (fallCount >= 2)
                {
                    endingAngle = angle;

                    objArr[numObj].number = numObj;
                    objArr[numObj].angle = (startingAngle + endingAngle) / 2; // or peakAngle
                    objArr[numObj].widthAng = endingAngle - startingAngle;
                    objArr[numObj].peakIR = peakIR;

                    numObj++;

                    inObject = 0;
                    fallCount = 0;
                    riseCount = 0;
                    peakIR = 0;
                    peakAngle = 0;
                }
            }
            else
            {
                // still inside object normally
                fallCount = 0;
            }
        }

        irPrev = irAverage;

        // Sending data to GUI through socket
        sprintf(buffer, "%d,%.2f,%d\n", angle, pingDist, irAverage);

        sendString(buffer);

        timer_waitMillis(20);

        receivedChar = returnChar;
        if (receivedChar == 'h')
        {
            break;
        }
    }

    //-----------Logic for object at 180 deg (not done scanning it)--------
    if (inObject == 1)
    {
        endingAngle = angle;

        objArr[numObj].number = numObj;
        objArr[numObj].angle = (startingAngle + endingAngle) / 2;
        objArr[numObj].widthAng = endingAngle - startingAngle;

        numObj++;

        inObject = 0;
    }

    int i;
    // second scan for PING dist
    for (i = 0; i < numObj; i++)
    {
        servo_move(objArr[i].angle);
        timer_waitMillis(200);

        objArr[i].distance_cm = read_ping_cm();
    }

    int j;
    int b;
    b = 1;
    if (numObj == 0)
    {
        object_count = 0;
        sendString("\r\nNo objects found.\r\n");
        sendString("SCAN_END\n");
        return;
    }
    struct object smallestObj = objArr[0];

    // ---- Print Object Data to terminal ----
    for (j = 0; j < numObj; j++)
    {
        objArr[j].widthLin = 2 * objArr[j].distance_cm * tan((objArr[j].widthAng * 3.14159268 / 180.0) / 2.0);
        if (objArr[j].widthAng < 5)
        {
            continue;
        }
        if (objArr[j].widthLin > 100 || objArr[j].widthLin < 0)
        {
            continue;
        }
        if (objArr[j].angle > 180)
        {
            continue;
        }
        if (objArr[j].distance_cm > 1000 || objArr[j].distance_cm < 0)
        {
            continue;
        }
        if (objArr[j].widthLin < smallestObj.widthLin)
        {
            smallestObj = objArr[j];
        }

        sprintf(buffer,
                "\r\nObject %d\r\n"
                "Center Angle: %d deg\r\n"
                "Distance: %.2f cm\r\n"
                "Width: %.2f cm\r\n",
                b++,
                objArr[j].angle,
                objArr[j].distance_cm,
                objArr[j].widthLin);

        sendString(buffer);
    }

    // make field map - CHECK VARS
    object_count = 0;
    smallest_object_num = 0;
    for (m = 0; m < numObj; m++)
    {
        float angle2 = objArr[m].angle * M_PI / 180.0;
        map_x[m] = objArr[m].distance_cm * cos(angle2) + robot_x;
        map_y[m] = objArr[m].distance_cm * sin(angle2) + robot_y;
        object_count++;
        if (objArr[m].number == smallestObj.number)
        {
            smallest_object_num = m;
        }
    }

    /* Tell the GUI the scan is complete so it commits the readings. */
    sendString("SCAN_END\n");

    return;
}

/*-----Main-----*/
int main(void)
{
    //    ObjectAnalyzer(log_data);

    // Initializing/Calibrating

    oi_t *sensor_data = oi_alloc(); // allocating memory for the data
    lcd_init();
    timer_init();
    uart_interrupt_init();
    oi_init(sensor_data);
    adc_init();

//    cyBOT_init_Scan(0b0111);

    // Calibrate Servo
    // cyBOT_SERVO_cal();

//    right_calibration_value = 290500;
//    left_calibration_value = 1282750;

    right_cal_value = 6400;
    left_cal_value = 34844;

    servo_init();
    ping_init();

    lcd_printf("Send 'm'");

    while (1)
    {

        // Receiving interrupt char
//        while(receivedChar == lastChar){
//            receivedChar = returnChar;
//        }
        receivedChar = returnChar;

        if (receivedChar == 'm')
        {
            // scan and map environment
            scanStartAngle = 0;
            scanEndAngle = 180;
            scan_and_map(sensor_data);

            sendString("\r\nScan Complete\r\n");
            lcd_clear();
            lcd_printf("Done");
            returnChar = ' ';
        }
        else if (receivedChar == 'h')
        {
            sendString("\r\nDone!\r\n");
            returnChar = ' ';
        }else if(receivedChar == 'n'){
            // 0-90 scan
            scanStartAngle = 0;
            scanEndAngle = 95;
            scan_and_map(sensor_data);

            sendString("\r\nScan Complete\r\n");
            lcd_clear();
            lcd_printf("Done");
            returnChar = ' ';

        }else if(receivedChar == 'l'){
            // 90-180 scan
            scanStartAngle = 90;
            scanEndAngle = 180;
            scan_and_map(sensor_data);

            sendString("\r\nScan Complete\r\n");
            lcd_clear();
            lcd_printf("Done");
            returnChar = ' ';

        }else{
            movement_update(sensor_data);
        }
        lastChar = receivedChar;

    }
    oi_free(sensor_data); // free memory
}
