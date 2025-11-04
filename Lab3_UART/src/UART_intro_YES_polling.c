#include <stdint.h>
#include "tm4c123gh6pm.h"

/* Function Prototypes */
void UART0_Init(void);
void GPIOA_UART0_Init(void);
void UART0Tx(char const c);
void delayMs(int n);

int main(void) 
{
    UART0_Init();           // Initialize UART0 peripheral
    GPIOA_UART0_Init();     // Initialize GPIOA pins for UART0
    
    delayMs(25);            // Wait for output line to stabilize

    while (1)
    {
        UART0Tx('Y');
        UART0Tx('e');
        UART0Tx('s');
        UART0Tx(' ');
    }
}

/* Initialize UART0 peripheral */
void UART0_Init(void)
{
    SYSCTL->RCGCUART |= 1;    // Provide clock to UART0
    SYSCTL->RCGCGPIO |= 1;    // Enable clock to PORTA
    
    UART0->CTL = 0;           // Disable UART0
    UART0->IBRD = 104;        // Integer part of baud rate divisor (for 9600 baud)
    UART0->FBRD = 11;         // Fractional part of baud rate divisor
    UART0->CC = 0;            // Use system clock
    UART0->LCRH = 0x60;       // 8-bit, no parity, 1-stop bit, no FIFO
    UART0->CTL = 0x301;       // Enable UART0, TX, and RX
}

/* Configure GPIOA pins for UART0 TX (PA1) and RX (PA0) */
void GPIOA_UART0_Init(void)
{
    GPIOA->DEN = 0x03;        // Enable digital function for PA0, PA1
    GPIOA->AFSEL = 0x03;      // Enable alternate function for PA0, PA1
    GPIOA->PCTL = 0x11;       // Configure PA0 and PA1 for UART
}

/* UART0 Transmit: waits until buffer available, then writes character */
void UART0Tx(char const c) 
{
    while ((UART0->FR & 0x20) != 0) {}  // Wait until Tx buffer is not full
    UART0->DR = c;                      // Write byte
}

/* Delay in milliseconds using SysTick timer (16 MHz clock) */
void delayMs(int n)
{
    SysTick->LOAD = 16000 - 1;          // 1 ms interval at 16 MHz
    SysTick->VAL = 0;                   // Clear current value
    SysTick->CTRL = 0x5;                // Enable SysTick, use system clock

    for (int i = 0; i < n; i++) {
        while ((SysTick->CTRL & 0x10000) == 0) {} // Wait for COUNT flag
    }
    SysTick->CTRL = 0;                  // Stop SysTick
}
