# robo-bar
Robotic bartender developed as part of the TE2003B System Design on Chip course, using the FRDM-KL25Z.

# Robo-Bar Firmware Documentation

## Overview
This document explains the functionality and role of each key file in the Robo-Bar embedded system project built for the FRDM-KL25Z development board using the MCUXpresso IDE and FreeRTOS.

---

## 1. `system_config.{c,h}`
**Purpose**: Defines system-wide constants, hardware specs, state management, and global variables.

### Key Features:
- Defines the system's finite state machine (`INIT`, `IDLE`, `DISPENSING`, etc.).
- Enumerates system errors (e.g., `SENSOR_FAILURE`, `COMMUNICATION`).
- Declares macros for timing, safety, buffer sizes.
- Manages system uptime and state through global variables.

---

## 2. `gpio_config.{c,h}`
**Purpose**: Configures and manages all GPIO pins used for servos, sensors, LEDs, motors, keypad, and LCD.

### Subsystems Covered:
- **Servos (PWM pins):** Configured for 6 dispensing and 1 direction-control servo.
- **Sensors:** Water level sensors, position sensors, and emergency stop input.
- **Motors:** H-bridge pins for mixing and conveyor motors.
- **Keypad:** 4x4 matrix interface using GPIO (rows: output, columns: input with pull-up).
- **LCD:** Configures I2C pins.
- **LEDs:** RGB status indicator.

### Also Includes:
- Pin abstraction structures: `gpio_pin_config_extended_t`, `gpio_input_config_t`.
- Functions to toggle pins, scan keypad, and control motor direction/enabling.

---

## 3. `pwm_config.{c,h}`
**Purpose**: Sets up the TPM modules to generate PWM signals for the servo motors.

### Components:
- TPM0, TPM1, TPM2 initialized with correct frequency (50Hz for servos).
- Converts desired angle to PWM pulse width.
- Supports:
  - Angle positioning for 0–180° servos.
  - Speed control and direction for 360° servo.
  - Utility functions for enabling/disabling all servos.
  - `pwm_servo_sweep_test` for diagnostic sweeping.

### Structures:
- `servo_config_t`: Tracks pulse width, angle, and servo type.
- `pwm_channel_config_t`: Maps TPM channel resources.

---

## 4. `uart_config.{c,h}`
**Purpose**: Enables UART communication for debugging and system interaction.

### Key Features:
- Configurable baud rate (default: 115200).
- Internal TX/RX circular buffers for non-blocking comms.
- FreeRTOS-based mutex locking for thread-safe UART usage.
- Interrupt-driven transmit/receive.
- Debug utilities:
  - `uart_printf()`, `debug_printf()`
  - `DEBUG_PRINT_INFO/WARNING/ERROR` macros.
  - Level filtering via `debug_level_t`.
- Statistics collection for transmission reliability.

---

## 5. FreeRTOS Integration (Planned in Project Scope)
- Tasks will be defined using `xTaskCreate()` for:
  - Servo/motor scheduling.
  - Sensor monitoring.
  - UI scanning (LCD/Keypad).
  - UART log handling.
- The project foundation enables `vTaskStartScheduler()` once these are set.

---

## 6. `system_tasks.{c,h}`
**Purpose**: Implements and manages the main system tasks using FreeRTOS, coordinating all real-time logic of the Robo-Bar.

### Key Functions:

- **Servo control and liquid dispensing:** Manages precise servo movements that control ingredient dispensing and direction.
- **Sensor monitoring:** Reads and processes data from sensors such as water levels, positions, and emergency buttons.
- **Motor control:** Manages conveyor and mixing motors, including start, stop, and direction control.
- **User interface:** Handles keypad input and updates the LCD display with status and menu information.
- **System monitoring:** Oversees system health, memory usage, task states, and detects errors or critical events.
- **UART communication:** Provides a debugging and external control channel via serial communication.
- **LED status indicators:** Updates RGB lighting to reflect the Robo-Bar’s operational state (ready, dispensing, error, emergency, etc.).

### Technical Organization:

- Defines and creates multiple FreeRTOS tasks with specific priorities to ensure critical functions (like emergency handling) have precedence.
- Uses queues, semaphores, event groups, and timers to synchronize and communicate between tasks.
- Implements internal states and contexts to manage recipe execution, dispensing progress, and mixing.
- Includes interrupt handlers for stack overflow and memory allocation failures.
- Starts the FreeRTOS scheduler to concurrently run all tasks.

---

## 7. Main Application (`robo-bar-fw.c`)

**Purpose:**  
The `robo-bar-fw.c` file serves as the entry point of the Robo-Bar firmware, responsible for initializing all hardware peripherals, setting up FreeRTOS synchronization objects, creating initial test and monitoring tasks, and finally starting the FreeRTOS scheduler to run the system.

### Key Responsibilities:

- **Hardware Initialization:**
  - Configures system clocks to run at 48 MHz.
  - Sets up GPIO pins for LEDs, servos, sensors, motors, and keypad.
  - Initializes PWM modules for servo control.
  - Configures UART for debugging and communication.
  - Initializes I2C bus for the LCD display.

- **System Health Indication:**
  - Runs an initial LED test sequence to indicate power-on and system readiness.
  
- **FreeRTOS Setup:**
  - Creates essential synchronization primitives like queues, semaphores, event groups, and timers (e.g., system heartbeat timer).
  - Verifies successful creation of FreeRTOS objects, otherwise indicates critical error by blinking red LED.

- **Task Creation:**
  - Spawns several initial FreeRTOS tasks including:
    - System Initialization Task (`vTaskSystemInit`): Prints startup info and sets system state to idle.
    - LED Test Task (`vTaskLedTest`): Continuously cycles through LED colors as a system alive indicator.
    - Servo Test Task (`vTaskServoTest`): Sweeps servo angles back and forth to verify PWM and servo functionality.
    - System Monitor Task (`vTaskSystemMonitor`): Periodically checks system status, memory usage, and sensor inputs, reporting via UART.

- **Scheduler Launch:**
  - Starts the FreeRTOS scheduler (`vTaskStartScheduler`) to begin multitasking.
  - Includes fail-safe infinite loop with error indication if scheduler startup fails.

- **Error Handling & Interrupts:**
  - Implements hooks for stack overflow, malloc failures, idle state, and tick events.
  - Provides UART interrupt handler for simple commands (e.g., reset, status).
  - Defines a hard fault handler that signals critical faults with LED patterns and halts the system.

### Overall Role:

The `robo-bar-fw.c` file **boots and verifies the hardware**, creates fundamental test and monitor tasks to validate key system components, and starts the FreeRTOS kernel to enable the Robo-Bar firmware to run all its coordinated tasks concurrently and responsively.

---

If you want, I can help you craft this explanation into a Markdown snippet to insert directly into your README. Would you like that?

---

---

## 8. [`robo-bar-pinout.md`](docs/robo-bar-pinout.md)
**Purpose:**  
This document provides a comprehensive mapping of all physical pin connections used by the Robo-Bar project on the FRDM-KL25Z board.

### Content Includes:
- Detailed pin assignments for servos, sensors, motors, LEDs, keypad, and LCD I2C.
- Corresponding GPIO ports, pins, and PWM channel mappings.
- Clear guidance for wiring the hardware components to the microcontroller.
- Helps ensure correct hardware setup and eases debugging.

---