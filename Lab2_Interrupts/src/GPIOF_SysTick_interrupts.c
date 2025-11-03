#include "TM4C123GH6PM.h"
#include "stdbool.h"

#define RED   (1U << 1)
#define BLUE  (1U << 2)
#define GREEN (1U << 3)

void delayMs(int n);
void LED_init(void);
void SW1_init(void);
void SW2_init(void);
void GPIOF_InterruptInit(void);
void SysTick_init(uint32_t reloadValue);

unsigned int col = 0;
unsigned int colors[] = {0x08, 0x06};
uint8_t setLED(uint8_t r, uint8_t g, uint8_t b);
bool toggle = 0;

/* ---------- MAIN ---------- */
int main(void)
{
    LED_init();          // initialize LEDs
    SW1_init();          // initialize SW1
    SW2_init();          // initialize SW2
    GPIOF_InterruptInit();    // setup interrupts for switches
		SysTick_init(8000000 - 1);    // Configure SysTick for 1-second interval
    __enable_irq();      // global interrupt enable
	

    while (1)
    {

    }
}

/* ---------- INITIALIZATION FUNCTIONS ---------- */

// initialize Port F pins for onboard LEDs
void LED_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;   // enable clock to GPIOF
    while((SYSCTL->PRGPIO & 0x20) == 0) {};  // wait until ready

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

// initialize SW2 (PF0) as input with pull-up
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

// setLED
uint8_t setLED(uint8_t r, uint8_t g, uint8_t b) {
    GPIOF->DATA &= ~(RED | BLUE | GREEN);
    if (r) GPIOF->DATA |= RED;
    if (b) GPIOF->DATA |= BLUE;
    if (g) GPIOF->DATA |= GREEN;
	
		return GPIOF->DATA & (RED | BLUE | GREEN);  // return current LED state
}


/* configure interrupts for SW1 and SW2 */
void GPIOF_InterruptInit(void)
{
    GPIOF->IS  &= ~0x11;  // edge-sensitive and not level-sensitive for bit4 and bit0
    GPIOF->IBE &= ~0x11;  // single edge
    GPIOF->IEV &= ~0x11;  // falling edge
    GPIOF->ICR  = 0x11;   // clear any prior interrupts
    GPIOF->IM  |= 0x11;   // unmask interrupts on PF4 and PF0

    NVIC->IP[30] = 3 << 5;        // set priority level 3
    NVIC->ISER[0] |= 0x40000000;  // enable IRQ30 for GPIOF
}

/* ---------- INTERRUPT HANDLER ---------- */
void GPIOF_Handler(void)
{
    volatile int readback;
    col = !col;
    GPIOF->ICR |= 0x11;  // clear interrupt flags
    readback = GPIOF->ICR; // dummy read to complete clear
}

/* ---------- DELAY FUNCTION ---------- */
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


void SysTick_init(uint32_t reloadValue)
{
    SysTick->LOAD = reloadValue;   // Set reload value (timer period)
    SysTick->VAL = 0;              // Clear current value
    SysTick->CTRL = 0x07;          // Enable SysTick with interrupt & system clock
}

/* ---------- Interrupt Handler ---------- */
void SysTick_Handler(void)
{
    
		if(toggle){
			setLED(0,0,0);           // Toggle red LED (PF1)
			toggle = 0;
		}else{
			GPIOF->DATA ^= colors[col];           // Toggle red LED (PF1)
			toggle = 1;
		}
}

