# STM32 Ultrasonic Sensor with LCD and LED Alert using STM32CubeIDE


## Features

- Sends trigger pulses to the ultrasonic sensor
- Reads the echo signal and calculates distance
- Displays the distance on an I2C 16x2 LCD
- Blinks a warning LED depending on how close the object is

## Setup

- STM32F4 (I used STM32F407VGT6 board)
- HC-SR04 Ultrasonic sensor
- 16x2 LCD with I2C module (address `0x27`)
- LED connected to pin `PA8`
- Breadboard and jumper wires
