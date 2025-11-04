#include <stdint.h>
#include "tm4c123gh6pm.h"

void delayMs(int n);

// Convenience functions
void UART0_init(void);
void UART0_sendMessage(const char* msg);

char const uart_message[] = "YES ";
volatile uint8_t tx_pos = 0;

int main(void)
{
    delayMs(30); // small delay to stabilize UART lines
    UART0_init(); // initialize UART0
    delayMs(30); // small delay to stabilize UART lines

    // Transmit first character to start TX interrupt-driven sending
    UART0->DR = uart_message[tx_pos++];

    while (1) {
        // main loop does nothing; TX handled by interrupt
    }
}

void UART0_init(void)
{
    // Enable clocks
    SYSCTL->RCGCUART |= 1; // UART0
    SYSCTL->RCGCGPIO |= 1; // GPIOA

    // UART0 setup
    UART0->CTL = 0;       // disable during setup
    UART0->IBRD = 104;    // integer part for 9600 baud
    UART0->FBRD = 11;     // fractional part
    UART0->CC = 0;        // system clock
    UART0->LCRH = 0x60;   // 8N1, no FIFO
    UART0->CTL = 0x301;   // enable UART0, TX, RX

    // GPIOA setup for UART0
    GPIOA->DEN = 0x03;    // PA0, PA1 digital enable
    GPIOA->AFSEL = 0x03;  // alternate function
    GPIOA->PCTL = 0x11;   // UART0 RX/TX

    // Interrupt setup
    UART0->IM |= 0x20;          // enable TX interrupt
    NVIC->ISER[0] |= (1 << 5);  // enable IRQ5
    __enable_irq();              // global IRQ enable
}

// UART0 transmit interrupt handler
void UART0_Handler(void)
{
    if (UART0->MIS & 0x20) { // TX interrupt
        UART0->DR = uart_message[tx_pos++];
        if (tx_pos == sizeof(uart_message) - 1) {
            tx_pos = 0; // wrap around
        }
        UART0->ICR = 0x20; // clear interrupt flag
    }
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