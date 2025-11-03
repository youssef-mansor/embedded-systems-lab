#include "TM4C123GH6PM.h"
#include <stdbool.h>

#define RED   (1U << 1)
#define BLUE  (1U << 2)
#define GREEN (1U << 3)

/* ---------- Function Prototypes ---------- */
void LED_init(void);
void SW1_init(void);
void SW2_init(void);
void GPIOF_InterruptInit(void);
void SysTick_init(uint32_t ms);
uint8_t setLED(uint8_t r, uint8_t g, uint8_t b);

/* ---------- Global Variables ---------- */
uint8_t currentColor = GREEN;
uint8_t previousColor = GREEN;
bool pause = false;

/* ---------- Main ---------- */
int main(void)
{
    LED_init();
    SW2_init();
    GPIOF_InterruptInit();
    SysTick_init(1000);         // 1-second SysTick tick
    __enable_irq();             // enable global interrupts

    setLED(0, 1, 0);            // start with green LED

    while (1)
    {
        // nothing here — all logic runs in ISRs
    }
}

/* ---------- Initialization ---------- */
void LED_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;
    while ((SYSCTL->PRGPIO & 0x20) == 0);

    GPIOF->DIR |= 0x0E;
    GPIOF->DEN |= 0x0E;
}

void SW1_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;
    while ((SYSCTL->PRGPIO & 0x20) == 0);

    GPIOF->DIR &= ~0x10;
    GPIOF->DEN |= 0x10;
    GPIOF->PUR |= 0x10;
}

void SW2_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;
    while ((SYSCTL->PRGPIO & 0x20) == 0);

    GPIOF->LOCK = 0x4C4F434B;
    GPIOF->CR |= 0x01;
    GPIOF->LOCK = 0;

    GPIOF->DIR &= ~0x01;
    GPIOF->DEN |= 0x01;
    GPIOF->PUR |= 0x01;
}

void GPIOF_InterruptInit(void)
{
    GPIOF->IS  &= ~0x11;  // edge-sensitive
    GPIOF->IBE &= ~0x11;  // single edge
    GPIOF->IEV &= ~0x11;  // falling edge
    GPIOF->ICR  = 0x11;   // clear prior interrupts
    GPIOF->IM  |= 0x11;   // unmask interrupts for PF4, PF0

    NVIC->IP[30] = 3 << 5;        // priority 3
    NVIC->ISER[0] |= 1 << 30;     // enable IRQ30 (GPIOF)
}

void SysTick_init(uint32_t ms)
{
    SysTick->LOAD = 16000 * ms - 1;   // ms delay at 16 MHz
    SysTick->VAL = 0;
    SysTick->CTRL = 0x07;             // enable SysTick + interrupt + system clock
}

/* ---------- LED Helper ---------- */
uint8_t setLED(uint8_t r, uint8_t g, uint8_t b)
{
    GPIOF->DATA &= ~(RED | BLUE | GREEN);
    if (r) GPIOF->DATA |= RED;
    if (b) GPIOF->DATA |= BLUE;
    if (g) GPIOF->DATA |= GREEN;
    return GPIOF->DATA & (RED | BLUE | GREEN);
}

/* ---------- Interrupt Service Routines ---------- */
void SysTick_Handler(void)
{
    static uint32_t counter = 0;

    if (pause)
        return; // traffic paused — skip changes

    counter++;

    if (currentColor == GREEN && counter >= 3)
    {
        counter = 0;
        previousColor = currentColor;
        currentColor = setLED(0, 0, 1); // blue
    }
    else if (currentColor == BLUE)
    {
        if (previousColor == GREEN && counter >= 1)
        {
            counter = 0;
            previousColor = currentColor;
            currentColor = setLED(1, 0, 0); // red
        }
        else if (previousColor == RED && counter >= 1)
        {
            counter = 0;
            previousColor = currentColor;
            currentColor = setLED(0, 1, 0); // green
        }
    }
    else if (currentColor == RED && counter >= 3)
    {
        counter = 0;
        previousColor = currentColor;
        currentColor = setLED(0, 0, 1); // blue
    }
}

void GPIOF_Handler(void)
{
    if (GPIOF->MIS & 0x01)
    {
        pause = !pause;   // toggle pause on SW2
        GPIOF->ICR = 0x01;
    }

    if (GPIOF->MIS & 0x10)
    {
        // SW1 not used — clear anyway
        GPIOF->ICR = 0x10;
    }
}
