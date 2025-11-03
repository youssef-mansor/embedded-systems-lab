#include "TM4C123GH6PM.h"

/* ---------- Function Prototypes ---------- */
void LED_init(void);
void SysTick_init(uint32_t reloadValue);

/* ---------- Main Function ---------- */
int main(void)
{
    LED_init();                    // Initialize LED pins (PF1–PF3)
    SysTick_init(16000000 - 1);    // Configure SysTick for 1-second interval

    __enable_irq();                // Enable global interrupts

    while (1)
    {
        // All work is handled by SysTick interrupt
    }
}

/* ---------- Initialization Functions ---------- */
void LED_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;          // Enable clock to Port F
    while ((SYSCTL->PRGPIO & 0x20) == 0); // Wait until Port F is ready

    GPIOF->DIR |= 0x0E;                // PF1–3 as outputs (LEDs)
    GPIOF->DEN |= 0x0E;                // Enable digital function on PF1–3
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
    GPIOF->DATA ^= 0x02;           // Toggle red LED (PF1)
}
