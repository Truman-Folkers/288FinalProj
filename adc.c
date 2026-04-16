
#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
//#include "uart-interrupt.h"
#include <stdbool.h>
//#include "driverlib/interrupt.h"

void adc_init(){
    SYSCTL_RCGCADC_R |= 0x01;   // enable ADC0
    while((SYSCTL_PRADC_R & 0x01) == 0) {}

    SYSCTL_RCGCGPIO_R |= 0x02;  // enable Port B
    while((SYSCTL_PRGPIO_R & 0x02) == 0) {}


    GPIO_PORTB_DIR_R &= ~0x10;    // input
    GPIO_PORTB_AFSEL_R |= 0x10;   // alternate function
    GPIO_PORTB_DEN_R &= ~0x10;    // disable digital
    GPIO_PORTB_AMSEL_R |= 0x10;   // enable analog

    ADC0_ACTSS_R &= ~0x08;   // disable SS3 (we usually use SS3)

    ADC0_EMUX_R &= ~0xF000;  // SS3 = processor trigger

    ADC0_SSMUX3_R = 10;   // channel 10

    ADC0_SSCTL3_R = 0x06;   // IE0 + END0

    ADC0_ACTSS_R |= 0x08;
}

uint16_t adc_read(){

    ADC0_PSSI_R = 0x08;   // start SS3

    while((ADC0_RIS_R & 0x08) == 0) {}

    uint16_t result = ADC0_SSFIFO3_R & 0xFFF;

    ADC0_ISC_R = 0x08;

    return result;

}
