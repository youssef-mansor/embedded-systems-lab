Here’s a **concise and well-structured README** for your experiment:

---

# 🚦 Tiva C LaunchPad — GPIOF Interrupt Experiment

## Overview

This experiment demonstrates how to use **hardware interrupts** on the **TM4C123GH6PM (Tiva C LaunchPad)** to control LED color changes when switches (SW1 and SW2) are pressed.
The LEDs toggle colors automatically, and each button press triggers an interrupt handled by the CPU.

---

## 💡 Hardware Connections

| Pin | Function  | Description                         |
| --- | --------- | ----------------------------------- |
| PF1 | Red LED   | Active high                         |
| PF2 | Blue LED  | Active high                         |
| PF3 | Green LED | Active high                         |
| PF4 | SW1       | Active low (needs pull-up)          |
| PF0 | SW2       | Active low (needs unlock + pull-up) |

---

## ⚙️ Code Structure

### **Initialization Functions**

* `LED_init()`
  Configures PF1–PF3 as output pins for LEDs.

* `SW1_init()` / `SW2_init()`
  Configure PF4 and PF0 as inputs with internal pull-ups.
  PF0 requires unlocking before configuration.

* `GPIOF_InterruptInit()`
  Sets up GPIOF interrupts for SW1 (PF4) and SW2 (PF0).

* `delayMs()`
  Millisecond delay using SysTick timer (16 MHz system clock).

---

## 🧠 Interrupt Handling (Explained Simply)

Interrupts let the microcontroller **stop what it’s doing** and **instantly react** to an event — here, a button press.

### Function: `GPIOF_InterruptInit()`

```c
void GPIOF_InterruptInit(void)
{
    GPIOF->IS  &= ~0x11;  // edge-sensitive
    GPIOF->IBE &= ~0x11;  // single edge
    GPIOF->IEV &= ~0x11;  // falling edge
    GPIOF->ICR  = 0x11;   // clear any prior interrupts
    GPIOF->IM  |= 0x11;   // unmask interrupts on PF4 and PF0

    NVIC->IP[30] = 3 << 5;        // set priority level 3
    NVIC->ISER[0] |= 0x40000000;  // enable IRQ30 for GPIOF
}
```

### 🔍 Line-by-Line Explanation

| Line                     | Meaning                                                                              |                                                            |
| ------------------------ | ------------------------------------------------------------------------------------ | ---------------------------------------------------------- |
| `GPIOF->IS &= ~0x11;`    | Configure PF4 and PF0 as **edge-sensitive** — react on change, not constant level.   |                                                            |
| `GPIOF->IBE &= ~0x11;`   | Choose **one edge** instead of both rising and falling.                              |                                                            |
| `GPIOF->IEV &= ~0x11;`   | Select **falling edge** — trigger when button pressed (signal goes from high → low). |                                                            |
| `GPIOF->ICR = 0x11;`     | Clear any old interrupt flags before enabling new ones.                              |                                                            |
| `GPIOF->IM               | = 0x11;`                                                                             | **Unmask** interrupts — allow them to trigger.             |
| `NVIC->IP[30] = 3 << 5;` | Assign **priority level 3** to Port F interrupt (medium).                            |                                                            |
| `NVIC->ISER[0]           | = 0x40000000;`                                                                       | Enable **IRQ30** in NVIC (the CPU’s interrupt controller). |

### When SW1/SW2 is Pressed:

1. The button pulls PF4/PF0 **low** → falling edge detected.
2. The interrupt flag is set in `GPIOF`.
3. NVIC sees IRQ30 and jumps to `GPIOF_Handler()`.
4. Your handler code runs instantly (e.g., toggling LED color).

---

## 🔄 Main Behavior

* LEDs alternate colors continuously.
* Pressing **SW1** or **SW2** triggers the **interrupt handler**.
* Inside `GPIOF_Handler()`, LED color toggles between two predefined patterns.

---

## 🧩 Example Flow

1. MCU starts → LEDs blink in a loop.
2. You press **SW1** → Port F interrupt fires.
3. `GPIOF_Handler()` executes and toggles the LED color.
4. Program returns to main loop — seamless and instant.

---

## 🧱 Delay Implementation

The delay is handled by **SysTick** for accurate timing:

```c
void delayMs(int n)
{
    SysTick->LOAD = 16000 - 1;      // 1 ms at 16 MHz
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;            // enable, use system clock
    for (int i = 0; i < n; i++)
        while ((SysTick->CTRL & 0x10000) == 0);
    SysTick->CTRL = 0;
}
```

---

## 🧾 Summary

| Concept          | What It Does                                      |
| ---------------- | ------------------------------------------------- |
| GPIOF Interrupts | Detect button presses instantly                   |
| SysTick Timer    | Provides precise millisecond delay                |
| NVIC             | Manages interrupt priorities and enables handlers |
| LEDs             | Provide visual feedback on interrupt events       |
