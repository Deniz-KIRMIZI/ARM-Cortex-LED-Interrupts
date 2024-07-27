# Interrupt-Driven LED Control on MKL25Z4

This repository contains the C code for implementing interrupt-driven LED control using the MKL25Z4 microcontroller. The project demonstrates handling of multiple interrupts and SysTick timer for real-time LED control and binary counter representation.

## Project Overview

The project utilizes the onboard RGB LED and four external LEDs connected to the GPIO ports. The system is controlled through interrupts triggered by button presses, manipulating LED states and displaying binary count values.

## Features

- **RGB LED Control:** Manipulate the color and blinking of the onboard RGB LED based on user input.
- **Binary Counter:** Increment a 4-bit binary counter displayed using four external LEDs.
- **Interrupt Handling:** Utilize interrupts to manage user inputs and system responses dynamically.

## Functionalities

- **SysTick Timer:** Leverages the SysTick timer for timing delays and precise control over LED states.
- **Button Inputs:** Uses three buttons to trigger different behaviors:
  - Button 1: Changes RGB LED to green and blinks it five times.
  - Button 2: Halts any ongoing LED operation for two seconds.
  - Button 3: Increments the binary counter across four LEDs.
- **Continuous Red Blinking:** When no button is pressed, the onboard red LED blinks continuously with a defined period.

## Code Structure

```c
#include "MKL25Z4.h"
#include "utils.h"  // Utility functions for initialization and delay

int main(void) {
    // Setup and initialization
    LedInitialization();
    InitializeCountByLED();
    SwitchOnboardInterruptsA();
    BinaryCounterInterrupt();

    // Main loop handling button presses and LED control
    while (1) {
        handleButtonPress();
        updateLEDState();
    }
}

void handleButtonPress(void) {
    // Check for button states and manage LED control logic
}

void updateLEDState(void) {
    // Update LED states based on system flags and counters
}

// Detailed implementations for initializing hardware and handling interrupts
void LedInitialization(void) {
    // Initialize LEDs for output
}

void InitializeCountByLED(void) {
    // Setup GPIO for binary counter display
}

void BinaryCounterInterrupt(void) {
    // Configure interrupts for counting
}
```

## Building and Running

- Compile the code using an ARM Cortex-M0 compatible compiler.
- Flash the compiled binary onto the MKL25Z4 microcontroller.
- Interact with the system using the buttons to observe different LED behaviors and counter states.


This README file provides a clear and succinct overview of the project, its functionalities, and how to get it running on a MKL25Z4 board. It’s intended to be practical for users who want to understand the project and potentially build upon it.
