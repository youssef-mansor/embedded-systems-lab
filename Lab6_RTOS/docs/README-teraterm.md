# STM32 UART Expression Calculator with FreeRTOS

## Project Overview
This project implements a UART-based expression calculator on the STM32 NUCLEO-L432KC board using FreeRTOS. The application receives simple arithmetic expressions from a PC via UART, evaluates them, and sends the results back.

## Hardware Requirements
- STM32 NUCLEO-L432KC Development Board
- USB-UART Bridge Cable/Module
- PC with Terminal Emulator (PuTTY, Tera Term, etc.)

## Software Requirements
- STM32CubeMX
- STM32CubeIDE
- FreeRTOS (CMSIS-RTOS v1)
- HAL Library

## UART Configuration
- **Baud Rate**: 115200
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None
- **Flow Control**: None

## Application Architecture

### Expression Format
The application accepts expressions in the format: `digit operator digit\r`
- **Valid digits**: 0-9
- **Valid operators**: + or -
- **Delimiter**: Carriage return (\r)
- **Example**: `5+3\r` returns `8`

### FreeRTOS Tasks

#### 1. Task3_UARTReceive
- **Entry Point**: `start_Task3_UARTReceive()`
- **Priority**: `osPriorityAboveNormal`
- **Stack Size**: 128 words
- **Function**: Handles UART interrupt processing using binary semaphore synchronization. Validates incoming characters, buffers the expression, and forwards it to the calculation task.

#### 2. Task1_Evaluate (calculation_task)
- **Entry Point**: `start_calculation_task()`
- **Priority**: `osPriorityNormal`
- **Stack Size**: 128 words
- **Function**: Reads characters from the receive queue, parses the expression, evaluates the arithmetic operation, and sends both the echoed expression and result to the print queue.

#### 3. Task2_UARTSend (print_task)
- **Entry Point**: `start_print_task()`
- **Priority**: `osPriorityLow`
- **Stack Size**: 128 words
- **Function**: Reads characters from the print queue and transmits them back to the PC via UART.

### Queues

#### ISR_to_calculation
- **Size**: 16 messages
- **Data Type**: `uint8_t`
- **Purpose**: Transfers received characters from Task3_UARTReceive to Task1_Evaluate

#### calculation_to_print
- **Size**: 16 messages
- **Data Type**: `uint8_t`
- **Purpose**: Transfers result characters from Task1_Evaluate to Task2_UARTSend

### Synchronization Primitives

#### uart_semaphore
- **Type**: Binary Semaphore
- **Purpose**: Synchronizes UART ISR with Task3_UARTReceive, ensuring minimal ISR execution time

## System Flow

1. **User Input**: User types expression (e.g., `7-2`) followed by Enter in terminal
2. **UART ISR**: `HAL_UART_RxCpltCallback()` receives character interrupt, disables UART interrupt, and gives semaphore
3. **Task3_UARTReceive**: Wakes up on semaphore, validates and buffers character, sends to queue, re-enables UART interrupt
4. **Task1_Evaluate**: Receives complete expression, evaluates it, sends result to output queue
5. **Task2_UARTSend**: Transmits echoed expression and result back to terminal

## Key Design Features

### Deferred Interrupt Processing
The UART ISR is kept minimal—it only disables the interrupt and releases a semaphore. All processing logic is moved to Task3_UARTReceive, following embedded systems best practices.

### Interrupt Management
- UART interrupt disabled before giving semaphore (`HAL_NVIC_DisableIRQ(USART2_IRQn)`)
- Re-enabled after processing in Task3_UARTReceive (`HAL_NVIC_EnableIRQ(USART2_IRQn)`)
- Prevents race conditions during character processing

### Task Priority Rationale
- **Task3_UARTReceive** (Highest): Ensures immediate response to incoming UART data
- **Task1_Evaluate** (Medium): Processes expressions with normal priority
- **Task2_UARTSend** (Lowest): Output can tolerate slight delays

## Building and Running

1. Open the project in STM32CubeIDE
2. Build the project (Ctrl+B)
3. Flash to STM32 NUCLEO-L432KC board
4. Open terminal emulator with 115200 baud rate
5. Type expressions like `3+4` and press Enter
6. Observe the echoed expression and calculated result

