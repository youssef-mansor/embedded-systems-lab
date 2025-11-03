/* This program blinks the red LED on the 
 * TI Tiva LaunchPad. The connections are: 
 * PF1 - red LED 
 * PF2 - blue LED 
 * PF3 - green LED 
 * They are high active (a '1' turns on the LED). 
 */ 
#include "TM4C123GH6PM.h"
#include <stdbool.h>   // for bool, true, false


void delayMs(int n);
void LED_init(void);
void SW1_init(void);
void SW2_init(void);
uint16_t currentColor = 0x02;
bool SW1_pressed(void);
bool SW2_pressed(void);

int main(void) 
{ 
    LED_init();  // initialize GPIOF for LEDs
		SW1_init();
		SW2_init();
	
		bool pause = 0;
	
    while(1) 
    { 
        if(SW1_pressed()){
					if(currentColor == 0x02)
						currentColor = 0x04;
					else if (currentColor == 0x04)
						currentColor = 0x08;
					else
						currentColor = 0x02;
				}
				
				if(pause){
					GPIOF->DATA = 0;
				} else {
					delayMs(250);
					GPIOF->DATA = currentColor;
					delayMs(250);
					GPIOF->DATA = 0;
				}
				if(SW2_pressed()){
					pause = !pause;
					delayMs(100);
				}
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

// initialize SW1 (PF4) as input with pull-up
void SW1_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;        // enable clock to GPIOF
    while((SYSCTL->PRGPIO & 0x20) == 0) {}; // wait until ready

    GPIOF->DIR &= ~0x10;             // set PF4 as input
    GPIOF->DEN |= 0x10;              // enable digital function on PF4
    GPIOF->PUR |= 0x10;              // enable pull-up resistor on PF4
}

void SW2_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;        // enable clock to GPIOF
    while((SYSCTL->PRGPIO & 0x20) == 0) {}; // wait until ready

    GPIOF->LOCK = 0x4C4F434B;        // unlock commit register
    GPIOF->CR |= 0x01;               // allow changes to PF0

    GPIOF->DIR &= ~0x01;             // set PF0 as input
    GPIOF->DEN |= 0x01;              // enable digital function on PF0
    GPIOF->PUR |= 0x01;              // enable pull-up resistor on PF0
}

bool SW1_pressed(void)
{
    // SW1 = PF4 (active low)
    return ((GPIOF->DATA & 0x10) == 0);
}

bool SW2_pressed(void)
{
    // SW2 = PF0 (active low)
    return ((GPIOF->DATA & 0x01) == 0);
}

// delay in milliseconds (16 MHz CPU clock)
void delayMs(int n)
{
    SysTick->LOAD = n * 15999;
    SysTick->CTRL = 0x5; // Enable timer with system clock
    while ((SysTick->CTRL & 0x10000) == 0); // Wait for COUNT flag
    SysTick->CTRL = 0; // Stop the timer
}