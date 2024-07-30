# STM32 AI-Powered Room Monitoring System

This project implements an AI-powered room monitoring system on an STM32 microcontroller. The system uses a sensor to measure various environmental parameters and predicts the occupancy status of a room. The system leverages a neural network model to make predictions based on sensor data.

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Project Structure](#project-structure)
- [Setup](#setup)
- [Usage](#usage)
- [Customization](#customization)
- [Troubleshooting](#troubleshooting)
- [License](#license)

## Hardware Requirements

- STM32 microcontroller (specific model not mentioned, but the project assumes support for I2C and UART peripherals)
- SCD30 CO2 sensor (or similar, for measuring CO2 concentration, temperature, and humidity)
- I2C interface for communication with the sensor
- UART interface for debugging and data output

## Software Requirements

- STM32CubeMX for hardware configuration and code generation
- X-CubeAI v9.0.0 software package installed in STM32CubeMX for Neural Network to C code conversion
- STM32CubeIDE or an equivalent IDE for developing and flashing the firmware
- HAL (Hardware Abstraction Layer) driver for STM32

## Project Structure

- `main.c`: The main source file containing the initialization and main sensing and predicting loop.
- `network.h`, `network_data.h`: Files related to the AI model, generated from STM32Cube.AI.
- `sensor.h`, `sensor.c`: Functions for interfacing with the SCD30 sensor.
- `ai_utils.c`, `ai_utils.h`: Utility functions for AI model inference.
- `stm32f4xx_hal_conf.h`, `system_stm32f4xx.c`: HAL configuration and system files.
- `README.md`: This documentation file.

## Setup

1. **Hardware Connection:**
   - Connect the SCD30 sensor to the I2C pins on the STM32 board.
   - Connect the UART TX pin to a serial-to-USB converter for debugging.

2. **Software Setup:**
   - Open the project in STM32CubeMX and configure the peripherals (I2C, UART, GPIO).
   - Generate the code and open it in STM32CubeIDE.
   - Add the AI model files (`network.h` and `network_data.h`) to the project.

3. **Building and Flashing:**
   - Build the project in STM32CubeIDE.
   - Flash the firmware onto the STM32 microcontroller.

## Usage

1. **Initialization:**
   - Upon startup, the system initializes the peripherals and the AI model.
   - The system will probe the SCD30 sensor. If successful, it will start periodic measurements.

2. **Data Collection:**
   - The system measures CO2 concentration, temperature, and humidity at regular intervals.
   - The data is processed and fed into the AI model to predict room occupancy.

3. **Output:**
   - The predicted occupancy status is printed via UART.
   - The prediction result is either "yes!" (occupied) or "no!" (not occupied).

## Customization

- **Sensor Data Simulation:** Uncomment the `FAKE_DATA` definition to use simulated data for testing.
- **Time Features:** Uncomment the `TIME_FEATURES` definition to include time-related features in the AI model input.
- **Decision Boundary:** Modify the `decision_boundary` variable in `main.c` to change the threshold for occupancy prediction.

## Troubleshooting

- **Sensor Probing Failed:** Ensure the SCD30 sensor is properly connected and powered.
- **UART Communication Issues:** Verify the UART settings and connections.
- **Model Inference Errors:** Check the AI model files and ensure they are correctly integrated.
- 
## Authors

- [@ismailessaidi11](https://www.github.com/ismailessaidi11)

## STM32CubeMX config 

![I2C parameters](./MX_config/I2C_parameters.png)

## License

This project is licensed under the terms provided in the LICENSE file, located in the root directory of this project. If no LICENSE file is present, the software is provided AS-IS, with no warranties or guarantees.
