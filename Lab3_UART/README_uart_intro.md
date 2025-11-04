Perfect 👍 Here’s your **updated README**, still simple and beginner-friendly — but now including the **math** behind the UART baud rate calculation for **IBRD** and **FBRD**.

---

## 🧠 Experiment: UART0 "Yes" Transmit on TM4C123 LaunchPad

### 🎯 Objective

This program demonstrates **how to send data from the TM4C123 (Tiva C LaunchPad)** to a computer or serial terminal using **UART0 (Universal Asynchronous Receiver/Transmitter)**.

In this example, the board repeatedly sends the word **"Yes "** over the UART0 serial interface.

---

## ⚙️ Hardware Setup

* **Microcontroller:** TM4C123GH6PM
* **UART0 TX (Transmit):** PA1
* **UART0 RX (Receive):** PA0
* Connect a USB cable between the LaunchPad and your computer — the UART0 pins are already connected internally to the USB interface.
* Open a serial terminal (like PuTTY or Tera Term) at **9600 baud**, **8 data bits**, **no parity**, **1 stop bit (8N1)**.

---

## 🧩 Code Overview

### 1. `main()` function

This is where the program starts:

1. Initializes the **UART0 peripheral**.
2. Configures **GPIO Port A** pins (PA0, PA1) for UART operation.
3. Waits a short time for stability.
4. Continuously sends the characters `'Y'`, `'e'`, `'s'`, and `' '` (space).

---

## ⚙️ UART Initialization Explained (Step by Step)

Let’s break down what happens inside the initialization, **in plain English**.

---

### 🕹️ 1. Enable Clocks

```c
SYSCTL->RCGCUART |= 1;   // Turn on clock for UART0
SYSCTL->RCGCGPIO |= 1;   // Turn on clock for Port A
```

✅ These lines **power up** the UART0 hardware and the GPIO port it uses (Port A).

---

### ⚙️ 2. Disable UART During Configuration

```c
UART0->CTL = 0;  // Temporarily disable UART0
```

✅ Disabling prevents UART from running while changing settings.

---

### 🧮 3. Set Baud Rate (Speed)

We want **9600 baud** communication (9,600 bits per second).
The TM4C UART baud rate is calculated using this formula:

[
\text{Baud Rate Divisor} = \frac{\text{System Clock}}{16 \times \text{Baud Rate}}
]

Given:

* System Clock = **16 MHz**
* Baud Rate = **9600**

[
\text{Baud Rate Divisor} = \frac{16{,}000{,}000}{16 \times 9600} = 104.1666
]

Now split it into:

* **Integer part (IBRD)** = 104
* **Fractional part (FBRD)** = (0.1666 × 64) + 0.5 = **11.16 ≈ 11**

So in code:

```c
UART0->IBRD = 104;   // Integer part
UART0->FBRD = 11;    // Fractional part
```

✅ Together, these make the UART transmit at approximately **9600 baud**.

---

### ⏱️ 4. Clock Source

```c
UART0->CC = 0;
```

✅ This tells UART to use the **system clock** (16 MHz).

---

### 📦 5. Frame Format

```c
UART0->LCRH = 0x60;
```

✅ This configures:

* 8 data bits
* 1 stop bit
* No parity
* No FIFO (simpler operation)

---

### 🔁 6. Enable UART0

```c
UART0->CTL = 0x301;
```

✅ Turns UART0 back on and enables both:

* **Transmitter (TXE, bit 8)**
* **Receiver (RXE, bit 9)**

Now UART0 is ready to send and receive data!

---

### 🧷 7. Configure GPIOA Pins for UART

```c
GPIOA->DEN = 0x03;    // Enable PA0, PA1 as digital pins
GPIOA->AFSEL = 0x03;  // Enable alternate function for UART
GPIOA->PCTL = 0x11;   // Assign UART0 to PA0 (RX) and PA1 (TX)
```

✅ This “connects” the physical pins (PA0, PA1) to the UART hardware.

---

### ✉️ 8. UART Transmission

```c
void UART0Tx(char const c) {
    while ((UART0->FR & 0x20) != 0) {}  // Wait until Tx buffer is empty
    UART0->DR = c;                      // Write character to data register
}
```

✅ The function waits until UART is ready, then sends one character.
The UART hardware automatically handles bit timing and transmission.

---

### ⏳ 9. Delay Function

```c
void delayMs(int n) {
    SysTick->LOAD = 16000 - 1;
    SysTick->VAL = 0;
    SysTick->CTRL = 0x5;
    for (int i = 0; i < n; i++)
        while ((SysTick->CTRL & 0x10000) == 0) {}
    SysTick->CTRL = 0;
}
```

✅ Creates a simple millisecond delay using the **SysTick timer**.

---

## 🧾 Summary

| Function             | Purpose                                            |
| -------------------- | -------------------------------------------------- |
| `UART0_Init()`       | Configure UART0 with 9600 baud (IBRD=104, FBRD=11) |
| `GPIOA_UART0_Init()` | Configure PA0/PA1 for UART RX/TX                   |
| `UART0Tx()`          | Send one character via UART                        |
| `delayMs()`          | Pause program for n milliseconds                   |
| `main()`             | Calls setup, then repeatedly sends `"Yes "`        |

---

## 💡 Expected Output

If you open a serial terminal at **9600 baud**, you should see:

```
Yes Yes Yes Yes ...
```

repeating continuously.

---

Would you like me to format this as a Markdown file (`README.md`) with proper code blocks and headings so you can directly include it in your lab folder?
