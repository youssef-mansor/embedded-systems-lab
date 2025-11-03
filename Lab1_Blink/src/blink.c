/* This program blinks the red LED on the 
 * TI Tiva LaunchPad. The connections are: 
 * PF1 - red LED 
 * PF2 - blue LED 
 * PF3 - green LED 
 * They are high active (a '1' turns on the LED). 
 */ 
#include "TM4C123GH6PM.h"

void delayMs(int n);
void LED_init(void);

int main(void) 
{ 
    LED_init();  // initialize GPIOF for LEDs

    while(1) 
    { 
        //GPIOF->DATA = 0x02;   // Red LED
        //GPIOF->DATA = 0x04;   // Blue LED
        GPIOF->DATA = 0x08;     // Green LED
        //GPIOF->DATA = 0x0A;   // Red + Green = Yellow
        
        delayMs(500); 
        GPIOF->DATA = 0;        // turn off LED
        delayMs(500); 
    } 
} 

// initialize Port F pins for onboard LEDs
void LED_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;   // enable clock to GPIOF
    while((SYSCTL->PRGPIO & 0x20) == 0) {};  // bit 5 = Port F

    GPIOF->DIR |= 0x0E;         // set PF1, PF2, PF3 as output
    GPIOF->DEN |= 0x0E;         // enable digital function on PF1–3
}

// delay in milliseconds (16 MHz CPU clock)
void delayMs(int n) 
{ 
    int i, j; 
    for(i = 0; i < n; i++)
        for(j = 0; j < 3180; j++)
            {} // do nothing for 1 ms
}
