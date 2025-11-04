#include "TM4C123GH6PM.h"

// Convenience functions
void UART0_init(void);
void UART1_init(void);
void UART1_RXIntInit(void);
void LED_init(void);
void UART0Tx(char c);

int main(void)
{
    UART0_init();        // Initialize UART0 for PC
    UART1_init();        // Initialize UART1 for phone
    LED_init();          // Initialize Port F LEDs
    UART1_RXIntInit();   // Enable UART1 receive interrupt and NVIC

    GPIOF->DATA = 0;     // Turn LEDs off

    __enable_irq();      // Global enable interrupts

    while (1) {}         // Main loop empty, everything handled in ISR
}

/* UART0 initialization for TX to PC */
void UART0_init(void)
{
    SYSCTL->RCGCUART |= 1; // Enable clock to UART0
    SYSCTL->RCGCGPIO |= 1; // Enable clock to Port A

    UART0->CTL = 0;        // Disable UART0
    UART0->IBRD = 104;     // Integer part: 16MHz/(16*9600)
    UART0->FBRD = 11;      // Fractional part: 0.1666*64 + 0.5
    UART0->CC = 0;         // Use system clock
    UART0->LCRH = 0x60;    // 8-bit, no parity, 1 stop, no FIFO
    UART0->CTL = 0x301;    // Enable UART0, TXE, RXE

    // Configure PA0/PA1 for UART0
    GPIOA->DEN |= 0x03;
    GPIOA->AFSEL |= 0x03;
    GPIOA->PCTL |= 0x11;
}

/* UART1 initialization for RX from phone */
void UART1_init(void)
{
    SYSCTL->RCGCUART |= 2; // Enable clock to UART1
    SYSCTL->RCGCGPIO |= 2; // Enable clock to Port B

    UART1->CTL = 0;        // Disable UART1
    UART1->IBRD = 104;     // Integer part: 16MHz/(16*9600)
    UART1->FBRD = 11;      // Fractional part: 0.1666*64 + 0.5
    UART1->CC = 0;         // Use system clock
    UART1->LCRH = 0x60;    // 8-bit, no parity, 1 stop, no FIFO
    UART1->CTL = 0x301;    // Enable UART1, TXE, RXE

    // Configure PB0/PB1 for UART1
    GPIOB->DEN |= 0x03;
    GPIOB->AFSEL |= 0x03;
    GPIOB->PCTL |= 0x11;
}

/* Enable UART1 RX interrupt and configure NVIC */
void UART1_RXIntInit(void)
{
    UART1->IM |= 0x0010;        // Enable RX interrupt
    NVIC->IP[6] = 3 << 5;       // Set IRQ6 priority to 3
    NVIC->ISER[0] |= 0x00000040;// Enable IRQ6 in NVIC
}

/* Initialize onboard LEDs on Port F (PF1–3) */
void LED_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;   // Enable clock to Port F
    while ((SYSCTL->PRGPIO & 0x20) == 0) {}; // Wait until ready

    GPIOF->DIR |= 0x0E;         // PF1–3 as output
    GPIOF->DEN |= 0x0E;         // Enable digital function
    GPIOF->DATA = 0;            // LEDs off
}

/* UART0 transmit */
void UART0Tx(char c)
{
    while (UART0->FR & 0x20);   // Wait while TX FIFO full
    UART0->DR = c;              // Send character
}

/* UART1 receive interrupt handler: forward to UART0 and update LEDs */
void UART1_Handler(void)
{
    volatile int readback;
    char c;

    if (UART1->MIS & 0x0010)  // Receive interrupt occurred
    {
        c = UART1->DR;         // Read received character
        UART0Tx(c);            // Forward to PC

        // Display on LEDs based on character
        switch (c)
        {
            case 'r': GPIOF->DATA = 0x02; break; // Red PF1
            case 'b': GPIOF->DATA = 0x04; break; // Blue PF2
            case 'g': GPIOF->DATA = 0x08; break; // Green PF3
        }

        UART1->ICR = 0x0010;     // Clear RX interrupt flag
        readback = UART1->ICR;   // Force clear
    }
    else
    {
        // Clear spurious interrupts if any
        UART1->ICR = UART1->MIS;
        readback = UART1->ICR;
    }
}
