/* This program blinks the red LED on the 
 * TI Tiva LaunchPad. The connections are: 
 * PF1 - red LED 
 * PF2 - blue LED 
 * PF3 - green LED 
 * They are high active (a '1' turns on the LED). 
 */ 
#include "TM4C123GH6PM.h"
#include <stdbool.h>   // for bool, true, false

#define RED   (1U << 1)
#define BLUE  (1U << 2)
#define GREEN (1U << 3)

void delayMs(int n);
bool delayMs_pause(int n);
void LED_init(void);
void SW1_init(void);
void SW2_init(void);
uint8_t currentColor = 0x02;
uint8_t previousColor = 0x02;
bool SW1_pressed(void);
bool SW2_pressed(void);
uint8_t setLED(uint8_t r, uint8_t g, uint8_t b);
bool pause = 0;


int main(void) 
{ 
    LED_init();  // initialize GPIOF for LEDs
		SW2_init();  // turn traffic light on / off
		currentColor = setLED(0,1,0); //green

    while(1) 
    { 
				
				if(pause){
					GPIOF->DATA = 0;
				} else {
					if(currentColor == GREEN){
						if (delayMs_pause(3000)) continue;
						previousColor = currentColor;
						currentColor = setLED(0,0,1); //blue
					} else if (currentColor == BLUE) {
						if (delayMs_pause(3000)) continue;
						if (previousColor == GREEN) {
							currentColor = setLED(1,0,0); //red
						} else if (previousColor == RED) {
							currentColor = setLED(0,1,0); //green
						}
					} else if (currentColor == RED){
						if (delayMs_pause(3000)) continue;
						previousColor = currentColor;
						currentColor = setLED(0,0,1); //blue
					}
					
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

uint8_t setLED(uint8_t r, uint8_t g, uint8_t b) {
    GPIOF->DATA &= ~(RED | BLUE | GREEN);
    if (r) GPIOF->DATA |= RED;
    if (b) GPIOF->DATA |= BLUE;
    if (g) GPIOF->DATA |= GREEN;
	
		return GPIOF->DATA & (RED | BLUE | GREEN);  // return current LED state
}


// delay in milliseconds (16 MHz CPU clock)

bool delayMs_pause(int n) {
    for (int i = 0; i < n; i++) {
        delayMs(1);
        if (SW2_pressed()) {
            pause = !pause;
            delayMs(100); // debounce
            return true;  // tell caller we paused
        }
    }
    return false; // completed without pause
}


void delayMs(int n)
{
    SysTick->LOAD = 16000 - 1;      // 1 ms interval at 16 MHz
    SysTick->VAL = 0;               // clear current value
    SysTick->CTRL = 0x5;            // enable SysTick, use system clock

    for (int i = 0; i < n; i++) {
        while ((SysTick->CTRL & 0x10000) == 0) {} // wait for COUNT flag, each iteration is 1ms
    }
    SysTick->CTRL = 0;              // stop SysTick
}


//void delayMs(int n)
//{
//    SysTick->LOAD = n * 15999;
//    SysTick->CTRL = 0x5; // Enable timer with system clock
//    while ((SysTick->CTRL & 0x10000) == 0); // Wait for COUNT flag
//    SysTick->CTRL = 0; // Stop the timer
//}

// delay in milliseconds (16 MHz CPU clock)
//void delayMs(int n) 
//{ 
//    int i, j; 
//    for(i = 0; i < n; i++)
//        for(j = 0; j < 3180; j++)
//            {} // do nothing for 1 ms
//}