#include <stdint.h>
#include <string.h>
#include "tm4c123gh6pm.h"

// LED definitions
#define RED   0x02
#define BLUE  0x04
#define GREEN 0x08
#define END_CHAR '*'

// UART/LED convenience functions
void UART0_Init(void);
void UART2_Init(void);
void UART0_Write(char c);
void UART0_Print(const char *s);
void LED_init(void);
void processCmd(const char *cmd);

// Globals
volatile char rxBuffer[32];
volatile uint8_t rx_pos = 0, cmd_flag = 0, rx_active = 0;

// --- main ---
int main(void)
{
    UART0_Init();      // Initialize UART0 (debug)
    UART2_Init();      // Initialize UART2 (phone)
    LED_init();        // Initialize PF LEDs

    UART0_Print("System Ready\r\n");

    while (1)
    {
        if (cmd_flag)
        {
            UART0_Print("\r\nParsed: ");
            UART0_Print((const char *)rxBuffer);
            UART0_Print("\r\n");
            processCmd((const char *)rxBuffer);
            cmd_flag = 0;
        }
    }
}

// --- UART0 functions ---
void UART0_Init(void)
{
    SYSCTL->RCGCUART |= 1;      // enable UART0
    SYSCTL->RCGCGPIO |= 1;      // enable PORTA

    GPIOA->DEN |= 0x03;         // digital enable PA0,PA1
    GPIOA->AFSEL |= 0x03;       // alternate function UART
    GPIOA->PCTL = (GPIOA->PCTL & ~0xFF) | 0x11; // UART0 RX/TX

    UART0->CTL = 0;
    UART0->IBRD = 104;
    UART0->FBRD = 11;
    UART0->LCRH = 0x60;
    UART0->CC = 0;
    UART0->CTL = 0x301;
}

void UART0_Write(char c)
{
    while (UART0->FR & 0x20);
    UART0->DR = c;
}

void UART0_Print(const char *s)
{
    while (*s) UART0_Write(*s++);
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

// --- LED functions ---
void LED_init(void)
{
    SYSCTL->RCGCGPIO |= 0x20;     // enable PORTF
    GPIOF->LOCK = 0x4C4F434B;
    GPIOF->CR |= 0x0E;
    GPIOF->DIR |= 0x0E;
    GPIOF->DEN |= 0x0E;
    GPIOF->DATA &= ~0x0E;         // all LEDs off
}

// --- UART2 interrupt handler ---
void UART2_Handler(void)
{
    char c;
    if (UART2->MIS & 0x10)
    {
        c = UART2->DR;
        UART0_Write(c);  // echo to debug

        if (!rx_active)
        {
            if (c == 'A') { rx_active = 1; rx_pos = 0; rxBuffer[rx_pos++] = c; }
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

// --- Command processing ---
void processCmd(const char *cmd)
{
    if      (!strcmp(cmd, "AT+R=1")) GPIOF->DATA |= RED;
    else if (!strcmp(cmd, "AT+R=0")) GPIOF->DATA &= ~RED;
    else if (!strcmp(cmd, "AT+G=1")) GPIOF->DATA |= GREEN;
    else if (!strcmp(cmd, "AT+G=0")) GPIOF->DATA &= ~GREEN;
    else if (!strcmp(cmd, "AT+P=1")) GPIOF->DATA |= (RED | BLUE);
    else if (!strcmp(cmd, "AT+P=0")) GPIOF->DATA &= ~(RED | BLUE);
    else if (!strcmp(cmd, "AT+Y=1")) GPIOF->DATA |= (RED | GREEN);
    else if (!strcmp(cmd, "AT+Y=0")) GPIOF->DATA &= ~(RED | GREEN);
    else if (!strcmp(cmd, "AT+ON"))  GPIOF->DATA |= (RED | GREEN | BLUE);
    else if (!strcmp(cmd, "AT+OFF")) GPIOF->DATA &= ~(RED | GREEN | BLUE);
}
