# ECE 2304 – Epileptic Seizure Detection System

Final project for ECE 2304 using two MSP432 microcontrollers communicating through UART.

## Project Overview

This project implements a wearable epileptic seizure detection and emergency response system using two MSP432 modules.

### Module A
- Reads accelerometer data using ADC
- Detects tonic-clonic seizure patterns
- Implements signal processing and classification
- Sends UART packets to Module B

### Module B
- Receives UART packets
- Handles alarms and caregiver interface
- Displays seizure information on LCD and 7-segment displays
- Controls buzzer, LEDs, and servo motor

## Features

- UART packet communication
- Real-time seizure classification
- Adjustable sensitivity thresholds
- Self-test mode
- Heartbeat communication monitoring
- LCD interface
- 7-segment seizure timer
- Servo warning indicator
- ADC + Timer interrupt sampling
- Low-power operation

## Technologies Used

- MSP432
- Embedded C
- UART
- ADC14
- Timers
- PWM
- Interrupts
- LCD driver
- State machines

## File Structure

```text
ModuleA/
    main.c
    lcdLib_432.c
    lcdLib_432.h

ModuleB/
    moduleB.c
```

## Authors

- Vianney Diaz
- Mauricio Aranda

## Course

ECE 2304 – Microcontroller Systems
University of Texas at El Paso