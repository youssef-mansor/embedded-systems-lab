#include "TM4C123GH6PM.h"
#include "stdbool.h"
#include <stdint.h>
#include <string.h>

#define RED   (1U << 1)
#define BLUE  (1U << 2)
#define GREEN (1U << 3)
#define END_CHAR '*'

void delayMs(int n);
void LED_init(void);
void GPIOF_InterruptInit(void);
void UART2_Init(void);
void SysTick_init(uint32_t reloadValue);
uint8_t setLED(uint8_t r, uint8_t g, uint8_t b);



// --- Global Variables ---
float factor = 1.0f;          // start at 1 Hz
const float step = 0.2f;      // change per button press
const float minHz = 0.2f;
const float maxHz = 5.0f;
bool toggle = 0;
volatile uint8_t rx_pos = 0, cmd_flag = 0, rx_active = 0;
volatile char rxBuffer[32];
uint8_t currentColor = 0x02;
uint8_t previousColor = 0x02;

// --- Convenience Functions ---
void Timer0_Init(void);
void GPIOF_Init(void);
void GPIOC_Init(void);
void processCmd(const char *cmd);


// --- Main ---
int main(void)
{
    // Initialize peripherals
    GPIOC_Init();      // PC4 LED
    GPIOF_Init();      // PF1 LED + PF0/SW2 and PF4/SW1
		SysTick_init(8000000 - 1);    // Configure SysTick for 0.5-second interval
		UART2_Init();      // Initialize UART2 (phone)
    __enable_irq();      // global interrupt enable

		delayMs(300);
    while (1)
    {
        if (cmd_flag)
        {
            processCmd((const char *)rxBuffer);
            cmd_flag = 0;
        }
    }
}

// --- Peripheral Initialization ---
void GPIOC_Init(void)
{
    SYSCTL->RCGCGPIO |= (1U << 2); // enable clock for Port C
    while ((SYSCTL->PRGPIO & (1U << 2)) == 0) {};
    GPIOC->DIR |= (1U << 4);       // PC4 output
    GPIOC->DEN |= (1U << 4);       // digital enable
    GPIOC->DATA &= ~(1U << 4);     // turn LED off
}

void GPIOF_Init(void)
{
    SYSCTL->RCGCGPIO |= (1U << 5); // enable clock for Port F
    while ((SYSCTL->PRGPIO & (1U << 5)) == 0) {};

    // Unlock PF0 (special case)
    GPIOF->LOCK = 0x4C4F434B;
    GPIOF->CR |= (1U << 0);
    GPIOF->LOCK = 0;

    // PF1 = LED output
    GPIOF->DIR |= (1U << 1);
    GPIOF->DEN |= (1U << 1);
    GPIOF->DATA &= ~(1U << 1);

    // PF0 = SW2 input, PF4 = SW1 input
    GPIOF->DIR &= ~((1U << 0) | (1U << 4));
    GPIOF->DEN |= ((1U << 0) | (1U << 4));
    GPIOF->PUR |= ((1U << 0) | (1U << 4));

    // Interrupt configuration: falling edge
    GPIOF->IS  &= ~((1U << 0) | (1U << 4));
    GPIOF->IBE &= ~((1U << 0) | (1U << 4));
    GPIOF->IEV &= ~((1U << 0) | (1U << 4));
    GPIOF->ICR  = ((1U << 0) | (1U << 4));
    GPIOF->IM  |= ((1U << 0) | (1U << 4));

    NVIC->ISER[0] |= (1U << 30);  // enable GPIOF interrupt
}

// --- UART2 functions ---
void UART2_Init(void)
{
    SYSCTL->RCGCUART |= (1 << 2); // UART2
    SYSCTL->RCGCGPIO |= (1 << 3); // PORTD

    GPIOD->LOCK = 0x4C4F434B;
    GPIOD->CR |= (1 << 7);        // unlock PD7
    GPIOD->DEN |= 0xC0;           // PD6, PD7 digital enable
    GPIOD->AFSEL |= 0xC0;         // alternate function
    GPIOD->PCTL = (GPIOD->PCTL & ~0xFF000000) | 0x11000000; // UART2 PD6/7

    UART2->CTL = 0;
    UART2->IBRD = 104;
    UART2->FBRD = 11;
    UART2->LCRH = 0x60;
    UART2->CC = 0;
    UART2->CTL = 0x301;
    UART2->IM |= 0x10;            // RX interrupt

    NVIC_EnableIRQ(UART2_IRQn);   // enable in NVIC
    __enable_irq();                // global enable
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



/* ---------- DELAY FUNCTION ---------- */

// delay in milliseconds (16 MHz CPU clock)
void delayMs(int n) 
{ 
    int i, j; 
    for(i = 0; i < n; i++)
        for(j = 0; j < 3180; j++)
            {} // do nothing for 1 ms
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
        GPIOF->DATA = 0;           // turn off PF LEDs
        GPIOC->DATA ^= (1U << 4); // toggle PC4
        toggle = 0;
    } else {
        GPIOF->DATA ^= currentColor;          // red LED on PF1
        GPIOC->DATA ^= (1U << 4); // toggle PC4
        toggle = 1;
    }
}

// --- Command processing ---
void processCmd(const char *cmd)
{
    currentColor = GREEN;
}
// --- UART2 interrupt handler ---
void UART2_Handler(void)
{
    char c;
    if (UART2->MIS & 0x10)
    {
        c = UART2->DR;

        if (!rx_active)
        {
            if (c == 't' || c == 'd') { rx_active = 1; rx_pos = 0; rxBuffer[rx_pos++] = c; }
        }
        else
        {
            if (c == END_CHAR)
            {
                rxBuffer[rx_pos] = '\0';
                cmd_flag = 1;
                rx_active = 0;
                rx_pos = 0;
            }
            else if (rx_pos < sizeof(rxBuffer) - 1)
            {
                rxBuffer[rx_pos++] = c;
            }
        }

        UART2->ICR = 0x10; // clear interrupt
    }
}
