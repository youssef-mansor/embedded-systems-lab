Here’s a **concise, clean README** for your traffic light experiment on the **TM4C123 Tiva LaunchPad**:

---

## 🚦 Traffic Light Control with Pause Feature – TM4C123 LaunchPad

### 🧠 Overview

This experiment simulates a **traffic light system** using the on-board RGB LED of the **TI Tiva TM4C123GH6PM** microcontroller.

* The LED cycles through **Green → Blue → Red → Blue → Green** repeatedly.
* **SW2 (PF0)** acts as a **pause/resume button**.

  * When pressed, the LEDs turn **off** (paused).
  * Press again to **resume** from the same state.

### ⚙️ Hardware Connections

| Function  | Port Pin | Color / Purpose                        |
| --------- | -------- | -------------------------------------- |
| Red LED   | PF1      | Red light                              |
| Blue LED  | PF2      | Blue light (transition)                |
| Green LED | PF3      | Green light                            |
| SW1       | PF4      | (Optional) Not used in this experiment |
| SW2       | PF0      | Pause/Resume button                    |

### 🧩 Main Features

* LED initialization function (`LED_init`)
* Switch initialization (`SW1_init`, `SW2_init`)
* LED control function (`setLED(r, g, b)`)
* Pause-aware delay (`delayMs_pause`)
* Debounce handling for reliable button press detection
* Uses **SysTick timer** for precise 1 ms timing

### 🕹️ Operation

1. Power and program the TM4C123 board with the provided code.
2. The LED will start cycling automatically:

   * Green → Blue → Red → Blue → Green
3. Press **SW2** to pause the cycle (LEDs turn off).
4. Press **SW2** again to resume.

### ⏱️ Timing

* Green and Red lights: 3 seconds each
* Blue light (transition): 3 seconds

### 🧱 Files

* **`main.c`** – Full implementation with initialization, LED control, and pause logic.

### 🧾 Notes

* The debounce delay is handled inside `delayMs_pause()`.
* `delayMs()` uses SysTick for accurate millisecond delays.
* You can easily adjust light durations or LED colors via `setLED()` calls.

---

Would you like me to format it in Markdown for a `README.md` file (ready to include in a GitHub repo)?
