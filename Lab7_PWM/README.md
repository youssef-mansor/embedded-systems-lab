# Frequency Generator with STM32

## Overview

This project generates PWM signals with frequencies ranging from 100 Hz to 2000 Hz using the **STM32** microcontroller and **Keil µVision**. The signal is output through **Timer1 (TIM1)** and can be connected to a speaker or audio device. The code cycles through different frequencies, producing a series of sound-like outputs.

## Hardware Requirements

* **STM32 Microcontroller** (e.g., STM32F103, STM32F4)
* **Speaker** (or an audio device)
* **USB-to-serial adapter** (optional for USART communication)
* **Power Supply** (3.3V or as per STM32 requirements)

## Pin Connections

| **Function**     | **STM32 Pin** | **Description**                             |
| ---------------- | ------------- | ------------------------------------------- |
| **PWM Output**   | `PA8`         | TIM1 Channel 1 for PWM output to speaker    |
| **USART2 TX**    | `PA2`         | UART Transmit Pin (for debugging, optional) |
| **USART2 RX**    | `PA3`         | UART Receive Pin (for debugging, optional)  |
| **Power Supply** | `VDD`         | 3.3V supply to STM32                        |
| **Ground**       | `GND`         | Ground pin                                  |

## Steps to Build and Run

1. **Set Up Keil µVision**:

   * Install [Keil µVision](https://www.keil.com/download/).
   * Create a new project, select your STM32 MCU, and configure peripherals using STM32CubeMX (optional).

2. **Copy Code**:

   * Paste the provided `main.c` code into your project.

3. **Build and Program**:

   * Build the project in Keil.
   * Flash the STM32 using a **ST-Link** or another programmer.

4. **Connect Hardware**:

   * **PA8 (PWM)**: Connect to a **speaker** or **audio device**.
   * **PA2 (TX)** and **PA3 (RX)**: Connect to a **USB-to-serial adapter** for debugging (optional).
   * **Power**: Connect **3.3V** to the power pin of STM32 and **GND** to ground.

5. **Test Output**:

   * The PWM signal will produce frequencies from 100 Hz to 2000 Hz.
   * If connected to a speaker, you should hear different frequencies.

## Code Explanation

* The code uses **Timer1 (TIM1)** to generate PWM signals. The frequencies are changed by adjusting the **prescaler** and **ARR (auto-reload)** values in a loop.
* Each frequency is held for **0.5 seconds** before moving to the next.

## Optional: Debugging with USART

* Use **USART2** for debugging. Connect **TX** (`PA2`) and **RX** (`PA3`) to a USB-to-serial adapter.
* Set the baud rate to **115200** in your terminal program.


