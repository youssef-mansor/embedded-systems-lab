# DS3231 RTC Alarms Overview

The **DS3231 Real-Time Clock (RTC)** includes two built-in alarms that
can trigger events at specific times or dates. These alarms allow a
microcontroller or system to respond automatically when a particular
moment occurs.

## Alarm Configuration

-   **Alarm 1** is stored in **registers 0x07 to 0x0A**.\
    These registers hold the alarm's seconds, minutes, hours, and
    day/date settings.

## How an Alarm Generates an Output

The DS3231 can produce an output signal on the **INT/SQW** pin when an
alarm condition is met.

This is controlled by two bits in the **Control Register (0x0E)**: -
**A1IE** --- Alarm 1 Interrupt Enable\
- **INTCN** --- Interrupt Control

An alarm will drive the **INT/SQW** pin low **only when**: 1. **A1IE =
1**, and\
2. **INTCN = 1**.

On power‑up, **A1IE = 0**, so alarms remain disabled until configured.

## Mask Bits (Flexible Alarm Matching)

Each alarm register includes a mask bit in **bit 7**: - Alarm 1 uses
**A1M1--A1M4**.

Mask bits determine how closely the alarm must match the current time:

-   **All mask bits = 0:**\
    Alarm triggers only when **seconds, minutes, hours, and day/date
    match exactly**.
-   **Some mask bits = 1:**\
    Selected fields are ignored, allowing repeat intervals such as every
    second, minute, hour, day, or date.

This provides flexibility for both single‑event alarms and periodic
alerts.
