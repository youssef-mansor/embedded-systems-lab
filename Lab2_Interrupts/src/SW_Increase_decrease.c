#include "TM4C123GH6PM.h"

// --- Global Variables ---
float factor = 1.0f;          // start at 1 Hz
const float step = 0.2f;      // change per button press
const float minHz = 0.2f;
const float maxHz = 5.0f;
uint32_t reloadVal;            // current reload value

// --- Convenience Functions ---
void Timer0_Init(void);
void GPIOF_Init(void);
void GPIOC_Init(void);
void update_timer_reload(void);

// --- Main ---
int main(void)
{
    // Initialize peripherals
    GPIOC_Init();      // PC4 LED
    GPIOF_Init();      // PF1 LED + PF0/SW2 and PF4/SW1
    Timer0_Init();     // Timer0A

    while (1)
    {
        // main loop does nothing, all handled by interrupts
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

void Timer0_Init(void)
{
    SYSCTL->RCGCTIMER |= (1U << 0); // enable Timer0
    while ((SYSCTL->PRTIMER & (1U << 0)) == 0) {};

    TIMER0->CTL = 0;         // disable Timer0A
    TIMER0->CFG = 0;         // 32-bit mode
    TIMER0->TAMR = 0x02;     // periodic down-counter
    reloadVal = (uint32_t)(16000000 / 2) / factor - 1;
    TIMER0->TAILR = reloadVal;
    TIMER0->ICR = 0x1;       // clear timeout
    TIMER0->IMR |= 0x1;      // enable interrupt

    NVIC->ISER[0] |= (1U << 19); // enable Timer0A interrupt
    __enable_irq();               // global IRQ enable
    TIMER0->CTL |= 0x01;          // start Timer0A
}

// --- Helper Functions ---
void update_timer_reload(void)
{
    reloadVal = (uint32_t)(16000000 / 2) / factor - 1;
    TIMER0->TAILR = reloadVal;
}

// --- Interrupt Handlers ---
void TIMER0A_Handler(void)
{
    TIMER0->ICR = 0x1;             // clear interrupt flag
    GPIOC->DATA ^= (1U << 4);      // toggle PC4
    GPIOF->DATA ^= (1U << 1);      // toggle PF1
}

void GPIOF_Handler(void)
{
    if (GPIOF->MIS & (1U << 0))    // SW2: decrease freq
    {
        if (factor - step >= minHz) { factor -= step; update_timer_reload(); }
        GPIOF->ICR = (1U << 0);
    }
    if (GPIOF->MIS & (1U << 4))    // SW1: increase freq
    {
        if (factor + step <= maxHz) { factor += step; update_timer_reload(); }
        GPIOF->ICR = (1U << 4);
    }
}

