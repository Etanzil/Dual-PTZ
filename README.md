# Camera Control System Firmware

This repository contains the firmware for a camera control system, enabling precise camera positioning and operation for various use cases, including day and thermal imaging. The firmware provides a robust set of features designed for efficient and reliable camera control.

## Features

### 1. Camera Initialization and Safety
- **Self-Testing Function**: Moves the camera to its origin during initialization.
- **Limit Switch Function**: Ensures the camera stays within its operational range.

### 2. UART Communication
- **Dual UARTs**: Supports communication with both day and night cameras.
- **Address Lines**:
  - **Address Line 1**: Recommended for thermal camera focusing.
  - **Address Line 2**: Suggested for day camera operations.

### 3. Preset Management
- **Preset Storage**: Stores up to 64 presets (32 for each camera).
- **Preset Operations**: 
  - **`OPERATION` Function**: Moves the camera to a specific preset.

### 4. Camera Motion Functions
- **Pan Function**: Allows 360° horizontal movement of the camera.
- **Scan Function**: Moves the camera horizontally within defined limits (left and right).
- **Diagonal Movement (`step_d`)**: Enables diagonal positioning of the camera.

### 5. Command Processing
- **`detect_cmd` Function**: Detects if a received command is valid.
- **`perform_task` Function**: Continuously executes the last received valid command.

### 6. Motor Control
- **360° Movement**: 
  - **Steps**: Completes 360° motion with 2000 motor steps.
  - **Range**: Operates within the range of "1637" to "2000" steps.
- **Ramp Function**: Gradually increases motor speed to prevent stalling.

## Getting Started

### Hardware Requirements
- Compatible microcontroller (e.g., STM32 series).
- Day and thermal cameras with UART communication support.
- Motor drivers and limit switches for motion control.

### Installation
1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/your-username/camera-control-system.git
   Build and flash the firmware using your preferred IDE Keil uvision 5
