
# Robo-Bar Project - Pinout and Connections (FRDM-KL25Z)

Este documento lista todas las conexiones de pines usadas en el proyecto Robo-Bar en la placa FRDM-KL25Z.

---

## Onboard RGB LEDs

| Color      | MCU Port | Pin Number | Description         |
|------------|----------|------------|---------------------|
| Red        | PORTB    | 18         | Red LED             |
| Green      | PORTB    | 19         | Green LED           |
| Blue       | PORTD    | 1          | Blue LED            |

---

## Relay Control for Pumps and Dispenser

| Relay            | MCU Port | Pin Number | Description             |
|------------------|----------|------------|-------------------------|
| Pump Relay 1     | PORTE    | 20         | Relay for ingredient pump 1 |
| Pump Relay 2     | PORTE    | 21         | Relay for ingredient pump 2 |
| Pump Relay 3     | PORTE    | 22         | Relay for ingredient pump 3 |
| Motor Relay      | PORTE    | 31         | Relay for motor control (mixing) |
| Dispenser Relay  | PORTE    | 23         | Relay for dispenser      |

---

## Water Level Sensors (Digital Inputs)

| Sensor      | MCU Port | Pin Number | Description            |
|-------------|----------|------------|------------------------|
| Sensor 1    | PORTC    | 1          | Water level sensor 1   |
| Sensor 2    | PORTC    | 2          | Water level sensor 2   |
| Sensor 3    | PORTC    | 3          | Water level sensor 3   |

---

## 4x4 Matrix Keypad

| Row         | MCU Port | Pin Number | Description        |
|-------------|----------|------------|--------------------|
| Row 1       | PORTB    | 0          | Keypad Row 1 (Output) |
| Row 2       | PORTB    | 1          | Keypad Row 2 (Output) |
| Row 3       | PORTB    | 2          | Keypad Row 3 (Output) |
| Row 4       | PORTB    | 3          | Keypad Row 4 (Output) |

| Column      | MCU Port | Pin Number | Description         |
|-------------|----------|------------|---------------------|
| Col 1       | PORTB    | 8          | Keypad Column 1 (Input, Pull-up) |
| Col 2       | PORTB    | 9          | Keypad Column 2 (Input, Pull-up) |
| Col 3       | PORTB    | 10         | Keypad Column 3 (Input, Pull-up)|
| Col 4       | PORTB    | 11         | Keypad Column 4 (Input, Pull-up)|

---

## 16x2 LCD Display (I2C Interface)

| Signal      | MCU Port | Pin Number | Description        |
|-------------|----------|------------|--------------------|
| I2C SDA     | PORTC    | 11         | LCD I2C Data Line  |
| I2C SCL     | PORTC    | 10         | LCD I2C Clock Line |

---

## Additional Sensors and Controls

| Device           | MCU Port | Pin Number | Description              |
|------------------|----------|------------|--------------------------|
| Emergency Stop   | PORTC    | 12         | Emergency stop button    |

---

## Motor Relay Control

| Signal        | MCU Port | Pin Number | Description              |
|---------------|----------|------------|--------------------------|
| Motor Enable  | PORTE    | 31         | Relay control for motor  |
