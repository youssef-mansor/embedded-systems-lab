#include <stdint.h>
#include "tm4c123gh6pm.h"

void UART0_init(void);
char UART0Rx(void);
void LED_init(void);
void delayMs(int n);

int main(void)
{
    char c;

    UART0_init();   // Initialize UART0 on Port A
    LED_init();     // Initialize LEDs on Port F

    GPIOF->DATA = 0; // Turn off LEDs

    for (;;)
    {
        c = UART0Rx();           // Receive character from UART
        GPIOF->DATA = c << 1;    // Display lower 3 bits on LEDs
    }
}

/* Initialize UART0 (PA0 = RX, PA1 = TX) */
void UART0_init(void)
{
    SYSCTL->RCGCUART |= 1;   // Enable clock to UART0
    SYSCTL->RCGCGPIO |= 1;   // Enable clock to Port A

    UART0->CTL = 0;          // Disable UART0
    UART0->IBRD = 104;       // Integer part: 16MHz / (16 * 9600) = 104.1666
    UART0->FBRD = 11;        // Fraction part: 0.1666 * 64 + 0.5 = 11
    UART0->CC = 0;           // Use system clock
    UART0->LCRH = 0x60;      // 8-bit, no parity, 1 stop bit, no FIFO
    UART0->CTL = 0x301;      // Enable UART0, TXE, RXEd

    // Configure PA0, PA1 for UART0
    GPIOA->DEN |= 0x03;      // Enable digital function
    GPIOA->AFSEL |= 0x03;    // Enable alternate function
    GPIOA->PCTL |= 0x11;     // Configure PA0, PA1 for UART
}

/* Initialize onboard LEDs on Port F (PF1–3) */
void LED_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;    // Enable clock to Port F
    while ((SYSCTL->PRGPIO & 0x20) == 0) {}; // Wait until ready

    GPIOF->DIR |= 0x0E;          // PF1–3 as output
    GPIOF->DEN |= 0x0E;          // Enable digital function
}

/* UART0 Receive — wait until a character is received, then return it */
char UART0Rx(void)
{
    while ((UART0->FR & 0x10) != 0) {}  // Wait while RX buffer is empty
    return (char)(UART0->DR);           // Read received data
}

/* Delay in milliseconds (16 MHz CPU clock) */
void delayMs(int n)
{
    SysTick->LOAD = 16000 - 1;      // 1 ms interval at 16 MHz
    SysTick->VAL = 0;               // Clear current value
    SysTick->CTRL = 0x5;            // Enable SysTick, use system clock

    for (int i = 0; i < n; i++)
    {
        while ((SysTick->CTRL & 0x10000) == 0) {} // Wait for COUNT flag
    }

    SysTick->CTRL = 0;              // Stop SysTick
}
