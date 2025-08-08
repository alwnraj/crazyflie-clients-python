# ESP32S3 - Crazyflie Connection Monitor Script

## Overview

`esp32_crazyflie_connection_monitor.py` is the main monitoring script that provides real-time monitoring and testing capabilities for the connection between a Xiao Sense ESP32S3 and Crazyflie using jumper cables.

## Purpose

This script helps debug and monitor the power connection between ESP32S3 and Crazyflie by:

- Detecting both devices automatically
- Establishing connections to both devices
- Monitoring connection status in real-time
- Testing power draw and voltage changes
- Providing interactive commands for testing
- Logging all activities for debugging

## Key Features

### 1. Automatic Device Detection

- **Crazyflie Detection**: Scans for Crazyflie devices using cflib
- **ESP32 Detection**: Scans serial ports for ESP32-related devices
- **Connection Status**: Tracks connection state for both devices

### 2. Real-time Monitoring

- Continuous status monitoring every 2 seconds
- Battery voltage tracking
- ESP32 communication monitoring
- Connection health checks

### 3. Interactive Testing

- Power connection testing
- ESP32 command sending
- Manual testing commands
- Real-time feedback

### 4. Comprehensive Logging

- All activities logged to `connection_log.txt`
- Console output for immediate feedback
- Timestamped entries for debugging

## Class Structure

### ConnectionMonitor Class

#### Initialization

```python
def __init__(self):
    self.crazyflie = None
    self.esp32_serial = None
    self.connection_status = {
        'crazyflie_connected': False,
        'esp32_connected': False,
        'communication_active': False
    }
    self.monitoring = False
```

#### Key Methods

##### `initialize_crazyflie_drivers()`

- **Purpose**: Initialize Crazyflie drivers and scan for devices
- **Returns**: List of available Crazyflie devices
- **Functionality**:
  - Initializes cflib drivers
  - Scans for available Crazyflie interfaces
  - Logs found devices
  - Handles initialization errors

##### `scan_esp32_ports()`

- **Purpose**: Scan for ESP32 serial ports
- **Returns**: List of ESP32-related serial ports
- **Functionality**:
  - Lists all available serial ports
  - Filters for ESP32-related devices (ESP32, Xiao, CH340, CP210)
  - Logs port information
  - Handles scanning errors

##### `connect_to_crazyflie(uri)`

- **Purpose**: Establish connection to Crazyflie
- **Parameters**: `uri` - Crazyflie device URI
- **Returns**: Boolean indicating connection success
- **Functionality**:
  - Creates Crazyflie object with callbacks
  - Sets up connection event handlers
  - Waits for connection with timeout
  - Handles connection failures

##### `connect_to_esp32(port, baudrate=115200)`

- **Purpose**: Establish serial connection to ESP32
- **Parameters**:
  - `port` - Serial port name
  - `baudrate` - Communication speed (default: 115200)
- **Returns**: Boolean indicating connection success
- **Functionality**:
  - Opens serial connection
  - Sends PING command to test communication
  - Verifies ESP32 response
  - Handles connection errors

##### `monitor_connections()`

- **Purpose**: Continuously monitor both connections
- **Functionality**:
  - Runs in separate thread
  - Updates status every 2 seconds
  - Reads ESP32 data when available
  - Logs connection status changes
  - Handles monitoring errors

##### `test_power_connection()`

- **Purpose**: Test power connection between devices
- **Returns**: Boolean indicating test success
- **Functionality**:
  - Verifies both devices are connected
  - Sends power test command to ESP32
  - Monitors battery voltage changes
  - Logs power test results

##### `send_esp32_command(command)`

- **Purpose**: Send commands to ESP32
- **Parameters**: `command` - Command string to send
- **Functionality**:
  - Sends command via serial connection
  - Logs command transmission
  - Handles communication errors

## Callback Functions

### Crazyflie Callbacks

- `_crazyflie_connected(uri)`: Handles successful Crazyflie connection
- `_crazyflie_disconnected(uri)`: Handles Crazyflie disconnection
- `_crazyflie_connection_failed(uri, msg)`: Handles connection failures

### Parameter Callbacks

- `_firmware_version_callback(name, value)`: Receives firmware version
- `_battery_voltage_callback(name, value)`: Receives battery voltage
- `_power_test_callback(name, value)`: Receives power test voltage

## Interactive Commands

### Available Commands

- `help` - Show available commands
- `test` - Test power connection between devices
- `power` - Send power test command to ESP32
- `quit` - Exit the program

### Command Processing

```python
while True:
    command = input("\nEnter command (help, test, power, quit): ").strip().lower()

    if command == 'quit':
        break
    elif command == 'help':
        # Show help
    elif command == 'test':
        monitor.test_power_connection()
    elif command == 'power':
        monitor.send_esp32_command("POWER_TEST")
```

## Error Handling

### Connection Errors

- Timeout handling for device connections
- Serial port access errors
- USB device detection failures
- Communication failures

### Monitoring Errors

- Thread-safe error handling
- Graceful degradation on failures
- Detailed error logging
- Recovery mechanisms

## Logging System

### Log Configuration

```python
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('connection_log.txt'),
        logging.StreamHandler()
    ]
)
```

### Log Levels

- **INFO**: Normal operation messages
- **WARNING**: Non-critical issues
- **ERROR**: Connection failures and errors
- **DEBUG**: Detailed debugging information

## Usage Examples

### Basic Usage

```bash
python esp32_crazyflie_connection_monitor.py
```

### Expected Output

```
============================================================
ESP32S3 - Crazyflie Connection Monitor
============================================================
This script will help you debug the connection between
your Xiao Sense ESP32S3 and Crazyflie via jumper cables.
Make sure you have connected:
  - 3V3 from ESP32 to VCC on Crazyflie breakout
  - GND from ESP32 to GND on Crazyflie breakout
============================================================

2024-01-01 12:00:00,000 - INFO - Initializing Crazyflie drivers...
2024-01-01 12:00:01,000 - INFO - Found 1 Crazyflie device(s)
2024-01-01 12:00:01,000 - INFO -   - radio://0/80/2M/E7E7E7E7E7
2024-01-01 12:00:01,000 - INFO - Scanning for ESP32 serial ports...
2024-01-01 12:00:01,000 - INFO -   - COM3: USB Serial Device (COM3)
2024-01-01 12:00:02,000 - INFO - ✓ Crazyflie connected successfully!
2024-01-01 12:00:02,000 - INFO - ✓ ESP32 responded: PONG
2024-01-01 12:00:02,000 - INFO - ESP32 connection successful!

Starting connection monitoring...
Press Ctrl+C to stop
------------------------------------------------------------

Enter command (help, test, power, quit):
```

## Dependencies

### Required Packages

- `cflib` - Crazyflie library for communication
- `pyserial` - Serial communication for ESP32
- `threading` - Multi-threading for monitoring
- `logging` - Logging system
- `time` - Timing and delays

### Installation

```bash
pip install -r requirements.txt
```

## Troubleshooting

### Common Issues

1. **No Crazyflie detected**: Check USB connection and drivers
2. **No ESP32 detected**: Check serial port and drivers
3. **Connection timeouts**: Verify device power and connections
4. **Communication errors**: Check baudrate and serial settings

### Debug Information

- Check `connection_log.txt` for detailed error information
- Verify device connections before running
- Test each device individually first
- Check system device manager for device recognition

## Safety Considerations

### Power Management

- Monitor battery voltage during tests
- Don't exceed current limits
- Ensure proper grounding
- Check for short circuits

### Connection Safety

- Always disconnect power before making connections
- Double-check connections before powering on
- Keep connections secure and stable
- Monitor for overheating

## Extensibility

### Adding New Commands

```python
elif command == 'new_command':
    # Add your custom functionality here
    monitor.send_esp32_command("CUSTOM_CMD")
```

### Adding New Monitoring

```python
def monitor_additional_sensor(self):
    # Add custom monitoring logic
    pass
```

### Custom Callbacks

```python
def custom_callback(self, name, value):
    # Handle custom parameter updates
    logger.info(f"Custom parameter {name}: {value}")
```

This script provides a comprehensive monitoring solution for debugging ESP32S3-Crazyflie connections with detailed logging, real-time monitoring, and interactive testing capabilities.
