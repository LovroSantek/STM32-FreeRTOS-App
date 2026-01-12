# STM32F4 FreeRTOS Audio & Control Application

This repository contains a FreeRTOS-based application for the **STM32F4DISCOVERY** development board.
The system integrates LED signaling, UART communication, sine-wave audio generation via DMA, and runtime parameter control through both UART commands and the user button.

---

## 1. Requirements

### Hardware

* STM32F4DISCOVERY board
* USB-mini cable (power and programming)

### Software

* Cross-compiler for STM32F4
* ST-Link flashing utility
* STM32CubeMX + STM32CubeIDE
* FreeRTOS source package

---

## 2. Application Overview

The application demonstrates coordinated real-time behavior across multiple FreeRTOS tasks, using UART, I2S, DMA, and GPIO peripherals. It provides real-time audio signal generation and configurable runtime control.

---

### Heartbeat LED Task

A dedicated FreeRTOS task drives the green LED in a heartbeat-style pattern.
The LED toggles according to a periodic 1-second signal, providing a simple indication that the system is running.

---

### UART Interface and Access Control

UART communication is implemented using **USART2** with DMA for both receiving and transmitting data.

On startup, the system sends a prompt requesting a password.
The correct password enables normal operation and triggers the message `OK`.
Incorrect input repeats the prompt.

---

### Sine Wave Audio Generation

Once the system enters normal operation, a **440 Hz sine wave** is generated and played through the audio output.
Audio streaming is performed using **I2S** with **HAL_I2S_Transmit_DMA()** to ensure non-blocking, continuous playback.

---

### Runtime Frequency and Amplitude Adjustment

Users can adjust audio output parameters via UART:

* `f%d` — set frequency (e.g., `f500` changes output to ~500 Hz)
* `a%f` — set amplitude (e.g., `a0.5` sets amplitude to 50%)

Commands are parsed by the UART task and forwarded to the audio task through a FreeRTOS message queue.
Valid commands yield `OK`, while invalid ones return `ERR`.

---

### Playback Control via User Button

The user button toggles the audio output state:

* Button press while audio is active → stop playback
* Button press while audio is stopped → resume playback

A timer-based debouncing mechanism ensures reliable button detection.
