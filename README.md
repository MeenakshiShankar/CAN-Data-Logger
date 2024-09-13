# FDCAN Communication with OpenAMP for Core-to-Core Data Exchange

## Overview

This project demonstrates inter-core communication between Cortex-M7 and Cortex-M4 cores using **OpenAMP** (Open Asymmetric Multi-Processing). The primary objective is to transfer sensor data collected on Cortex-M7 to Cortex-M4, which then transmits the data externally via **FDCAN** (Flexible Data-rate Controller Area Network). OpenAMP facilitates communication between the two cores through shared memory, while FDCAN is responsible for the transmission and reception of sensor data.

### Components Involved:
- **Cortex-M7**: Collects sensor data and forwards it to Cortex-M4 using OpenAMP.
- **Cortex-M4**: Receives data from Cortex-M7 and sends it via FDCAN.
- **FDCAN**: Peripheral handling CAN-based data transmission.
- **OpenAMP**: Middleware managing inter-core communication.

## Project Structure

- **Cortex-M7 (Core 1)**
  - Collects sensor data from a connected sensor.
  - Transmits sensor data to Cortex-M4 using OpenAMP (RPMsg communication).

- **Cortex-M4 (Core 2)**
  - Receives sensor data from Cortex-M7 using OpenAMP.
  - Transmits the received sensor data over the FDCAN peripheral.

### Key Features
- **FDCAN Peripheral**: Configured to communicate with external CAN devices.
- **OpenAMP**: Core-to-core communication using RPMsg channels.
- **Sensor Data Handling**: Data collected from sensors on Cortex-M7 is processed and transmitted externally by Cortex-M4.

## Hardware Requirements
- STM32H7 Microcontroller (Dual-core: Cortex-M7 and Cortex-M4)
- Sensors interfaced with Cortex-M7
- FDCAN transceiver module for CAN communication

## Software Requirements
- STM32CubeIDE or STM32CubeMX for configuration
- OpenAMP middleware for inter-core communication
- HAL libraries for FDCAN and UART

## Setup and Configuration

1. **GPIO Initialization**:
    - Both FDCAN and UART peripherals are initialized on Cortex-M4.
    - FDCAN uses a standard identifier, frame format, and baud rate.

2. **OpenAMP Initialization**:
    - OpenAMP enables communication between Cortex-M7 and Cortex-M4 using RPMsg and mailbox interrupts.

3. **FDCAN Configuration**:
    - FDCAN is configured in loopback mode for testing, and in standard mode for external communication.

## Code Flow

### Cortex-M7 Code Flow:
1. **Sensor Data Acquisition**: Sensor data is periodically collected from connected sensors.
2. **Sending Data to Cortex-M4**: The data is packaged and sent to Cortex-M4 through OpenAMP (RPMsg communication).
3. **OpenAMP Endpoint**: An RPMsg endpoint is created to send and receive data from Cortex-M4.

### Cortex-M4 Code Flow:
1. **Receiving Data from Cortex-M7**: Cortex-M4 continuously listens for messages sent by Cortex-M7 using OpenAMP.
2. **FDCAN Transmission**: Once data is received, Cortex-M4 transmits it externally via FDCAN.
3. **CAN Message Handling**: FDCAN is configured to manage message interrupts for receiving/transmitting messages.

### Functions of Interest

#### Cortex-M7
- `rpmsg_recv_callback()`: Handles messages received via RPMsg and forwards the sensor data to Cortex-M4.
- `OPENAMP_create_endpoint()`: Creates an RPMsg endpoint for communication with Cortex-M4.

#### Cortex-M4
- `HAL_FDCAN_TxFifoQAddMessage()`: Adds sensor data to the FDCAN transmission queue.
- `HAL_FDCAN_RxFifo0Callback()`: Handles receiving FDCAN messages.

## OpenAMP Communication

OpenAMP enables core-to-core communication by using shared memory for message passing. Here's an overview of how OpenAMP is used in this project:

1. **RPMsg Endpoints**: Both cores create RPMsg endpoints for sending and receiving data.
2. **Message Exchange**: Cortex-M7 sends the sensor data through the RPMsg channel, and Cortex-M4 processes this data and sends it over FDCAN.

## Usage

1. Clone or download the repository.
2. Open the project in **STM32CubeIDE**.
3. Flash the code onto the STM32H7 microcontroller.
4. Connect the CAN transceivers to the appropriate GPIO pins.
5. Use a CAN analyzer or FDCAN-compatible device to monitor the data transmission.

## Conclusion

This project demonstrates how to use **FDCAN** and **OpenAMP** to perform real-time core-to-core communication for sensor data transmission in an embedded system. The use of OpenAMP ensures smooth data flow between Cortex-M7 and Cortex-M4, while FDCAN handles external communication.
```
