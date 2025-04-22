# ESP-IDF Nanomodbus Master Example

This project demonstrates how to use the Nanomodbus library as a Modbus RTU master to communicate with a DDS661 energy meter using an ESP32.

## Features
- Reads multiple parameters from DDS661 Modbus RTU slave device
- Converts raw register values to floating-point measurements
- Prints formatted measurements with units to the serial console
- Uses hardware UART with RS485 transceiver control
- Includes relay control example

## Hardware Requirements
- ESP32 development board
- DDS661 energy meter (or compatible Modbus RTU device)
- RS485 transceiver (e.g., MAX485)
- Wiring connections:
  - ESP32 GPIO16 → RS485 RO (receiver out)
  - ESP32 GPIO17 → RS485 DI (driver in)
  - ESP32 GPIO4 → RS485 DE/RE (driver enable/receiver enable)
  - Relay connected to GPIO21

## Software Requirements
- ESP-IDF v5.3.2
- Nanomodbus library

## Configuration
- Modbus RTU slave address: 0x01 (configurable via `RTU_SERVER_ADDRESS`)
- Baud rate: 9600
- Parity: Even
- Stop bits: 1
- Flow control: None

## Register Map
The example reads the following registers from the DDS661:

| Address (Hex) | Description         | Unit           | Format         |
|---------------|---------------------|----------------|----------------|
| 0x0000        | Voltage             | V              | Floating Point |
| 0x0008        | Electric Current    | A              | Floating Point |
| 0x0012        | Active Power        | kW             | Floating Point |
| 0x002A        | Power Factor        | COS            | Floating Point |
| 0x0036        | Frequency           | Hz             | Floating Point |
| 0x0100        | Total Active Power  | kWh            | Floating Point |

## Usage
1. Connect your hardware as described above
2. Clone the repository
3. Configure the project (`idf.py menuconfig` if needed)
4. Build and flash the project (`idf.py build flash monitor`)
5. The ESP32 will:
   - Enable relay on GPIO21
   - Periodically read all configured registers
   - Print formatted measurements every 5 seconds

## Customization
- Change `RTU_SERVER_ADDRESS` to match your slave device address
- Modify `address[]`, `label[]`, and `unit[]` arrays to read different registers
- Adjust timing parameters in `nmbs_set_read_timeout()` and `nmbs_set_byte_timeout()`

## Troubleshooting
- Verify physical connections (UART pins, RS485 direction control)
- Check slave device address and baud rate configuration
- Monitor serial output for error messages
- Ensure proper termination on the RS485 bus

## Author
Rithy Lim - 2025-04-22