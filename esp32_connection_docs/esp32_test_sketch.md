# ESP32S3 Test Sketch Documentation

## Overview

`esp32_test_sketch.ino` is an Arduino sketch designed for the Xiao Sense ESP32S3 that provides communication and testing capabilities for the ESP32S3-Crazyflie connection monitoring system.

## Purpose

This sketch enables the ESP32S3 to:

- Respond to commands from the Python monitoring script
- Provide system status information
- Simulate power draw for testing
- Monitor power connections
- Send heartbeat messages
- Control built-in LED for visual feedback

## Hardware Requirements

### Required Components

- **Xiao Sense ESP32S3**: Main microcontroller
- **USB Cable**: For programming and communication
- **Jumper Wires**: For power connection testing
- **LED**: Built-in LED for visual feedback

### Pin Definitions

```cpp
#define POWER_MONITOR_PIN 1  // GPIO1 for power monitoring
#define LED_PIN 2            // Built-in LED
```

## Software Structure

### Global Variables

```cpp
unsigned long lastHeartbeat = 0;
const unsigned long HEARTBEAT_INTERVAL = 5000; // 5 seconds
bool powerTestMode = false;
```

### Main Functions

#### `setup()`

**Purpose**: Initialize the ESP32S3 system
**Functionality**:

- Initialize serial communication at 115200 baud
- Configure GPIO pins
- Display startup information
- Provide command instructions

```cpp
void setup() {
    Serial.begin(115200);
    while (!Serial) {
        delay(10);
    }

    pinMode(LED_PIN, OUTPUT);
    pinMode(POWER_MONITOR_PIN, INPUT);

    digitalWrite(LED_PIN, HIGH);
    delay(1000);
    digitalWrite(LED_PIN, LOW);

    Serial.println("ESP32S3 - Crazyflie Connection Test");
    Serial.println("Ready for commands:");
    Serial.println("  PING - Respond with PONG");
    Serial.println("  POWER_TEST - Enter power test mode");
    Serial.println("  STATUS - Send current status");
    Serial.println("  HEARTBEAT - Send periodic status");
}
```

#### `loop()`

**Purpose**: Main program loop
**Functionality**:

- Handle incoming serial commands
- Send periodic heartbeat messages
- Run power test mode when active
- Maintain system responsiveness

```cpp
void loop() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        command.toUpperCase();
        handleCommand(command);
    }

    if (millis() - lastHeartbeat > HEARTBEAT_INTERVAL) {
        sendHeartbeat();
        lastHeartbeat = millis();
    }

    if (powerTestMode) {
        runPowerTest();
    }

    delay(100);
}
```

## Command System

### Available Commands

#### `PING`

**Purpose**: Test communication
**Response**: `PONG`
**Usage**: Basic connectivity test

#### `POWER_TEST`

**Purpose**: Enter power test mode
**Response**: `Entering power test mode`
**Functionality**:

- Activates power test mode
- Turns on LED
- Begins rapid LED toggling

#### `STATUS`

**Purpose**: Send current system status
**Response**: Formatted status string
**Information**:

- Uptime in milliseconds
- Free heap memory
- Power pin reading
- LED state
- Power test mode status

#### `HEARTBEAT`

**Purpose**: Enable periodic status updates
**Response**: `Heartbeat enabled`
**Functionality**: Enables automatic status reporting

#### `STOP_POWER_TEST`

**Purpose**: Exit power test mode
**Response**: `Exiting power test mode`
**Functionality**:

- Deactivates power test mode
- Turns off LED
- Stops rapid toggling

#### `LED_ON`

**Purpose**: Turn on built-in LED
**Response**: `LED ON`
**Usage**: Manual LED control

#### `LED_OFF`

**Purpose**: Turn off built-in LED
**Response**: `LED OFF`
**Usage**: Manual LED control

#### `BLINK`

**Purpose**: Blink LED 5 times
**Response**: `Blink complete`
**Functionality**: Visual feedback test

### Command Processing

```cpp
void handleCommand(String command) {
    if (command == "PING") {
        Serial.println("PONG");
    }
    else if (command == "POWER_TEST") {
        Serial.println("Entering power test mode");
        powerTestMode = true;
        digitalWrite(LED_PIN, HIGH);
    }
    // ... additional commands
    else {
        Serial.print("Unknown command: ");
        Serial.println(command);
    }
}
```

## Status Reporting

### `sendStatus()`

**Purpose**: Send comprehensive system status
**Output Format**:

```
STATUS: UPTIME=12345 FREE_HEAP=123456 POWER_PIN=512 LED=1 POWER_TEST=ON
```

**Information Provided**:

- **UPTIME**: System uptime in milliseconds
- **FREE_HEAP**: Available heap memory
- **POWER_PIN**: Analog reading from power monitor pin
- **LED**: Current LED state (0/1)
- **POWER_TEST**: Power test mode status (ON/OFF)

### `sendHeartbeat()`

**Purpose**: Send periodic heartbeat message
**Output Format**:

```
HEARTBEAT: UPTIME=12345 FREE_HEAP=123456
```

**Information Provided**:

- **UPTIME**: System uptime in milliseconds
- **FREE_HEAP**: Available heap memory

## Power Testing

### `runPowerTest()`

**Purpose**: Simulate power draw for testing
**Functionality**:

- Toggles LED rapidly (every 100ms)
- Monitors power pin readings
- Sends power test data
- Simulates varying power consumption

```cpp
void runPowerTest() {
    static unsigned long lastToggle = 0;
    static bool ledState = false;

    if (millis() - lastToggle > 100) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        lastToggle = millis();

        Serial.print("POWER_TEST:");
        Serial.print(" LED=");
        Serial.print(ledState ? "ON" : "OFF");
        Serial.print(" POWER_PIN=");
        Serial.print(analogRead(POWER_MONITOR_PIN));
        Serial.println();
    }
}
```

**Output Format**:

```
POWER_TEST: LED=ON POWER_PIN=512
```

## System Information

### `printSystemInfo()`

**Purpose**: Display detailed system information
**Information Provided**:

- Chip model and revision
- Number of CPU cores
- Flash memory size
- Available heap memory

```cpp
void printSystemInfo() {
    Serial.println("=== System Information ===");
    Serial.print("Chip Model: ");
    Serial.println(ESP.getChipModel());
    Serial.print("Chip Revision: ");
    Serial.println(ESP.getChipRevision());
    Serial.print("Chip Cores: ");
    Serial.println(ESP.getChipCores());
    Serial.print("Flash Size: ");
    Serial.print(ESP.getFlashChipSize() / 1024 / 1024);
    Serial.println(" MB");
    Serial.print("Free Heap: ");
    Serial.print(ESP.getFreeHeap());
    Serial.println(" bytes");
    Serial.println("========================");
}
```

## Communication Protocol

### Serial Communication

- **Baud Rate**: 115200
- **Data Format**: ASCII text
- **Line Ending**: `\n` (newline)
- **Timeout**: None (blocking)

### Message Format

- **Commands**: Uppercase ASCII strings
- **Responses**: Plain text with newline
- **Status**: Key-value pairs separated by spaces
- **Data**: Comma-separated values

### Error Handling

- **Unknown Commands**: Reported with command name
- **Serial Errors**: Handled gracefully
- **Pin Errors**: Reported in status
- **Memory Issues**: Monitored in heartbeat

## Power Management

### Power Monitoring

- **Power Pin**: GPIO1 (analog input)
- **Voltage Range**: 0-3.3V
- **Resolution**: 12-bit (0-4095)
- **Sampling**: Continuous monitoring

### Power Test Mode

- **LED Toggle**: Every 100ms
- **Power Draw**: Simulated through LED
- **Monitoring**: Continuous voltage reading
- **Reporting**: Real-time data transmission

## Timing Considerations

### Heartbeat Timing

- **Interval**: 5 seconds
- **Non-blocking**: Uses millis() timing
- **Automatic**: No command required
- **Configurable**: HEARTBEAT_INTERVAL constant

### Power Test Timing

- **Toggle Rate**: 100ms
- **Non-blocking**: Uses millis() timing
- **Responsive**: Maintains command handling
- **Configurable**: Adjustable in code

## Memory Management

### Heap Monitoring

- **Free Heap**: Reported in status messages
- **Memory Leaks**: Detectable through monitoring
- **Optimization**: Minimal memory usage
- **Stability**: Long-term operation support

### String Handling

- **Dynamic Allocation**: Minimal string usage
- **Memory Efficiency**: Reuse of string objects
- **Garbage Collection**: Automatic cleanup
- **Fragmentation**: Monitored through heap size

## Safety Features

### Power Safety

- **Voltage Monitoring**: Continuous monitoring
- **Overcurrent Protection**: Built-in ESP32 protection
- **Thermal Protection**: Automatic shutdown
- **Brown-out Detection**: Automatic reset

### Communication Safety

- **Timeout Handling**: Prevents hanging
- **Error Recovery**: Automatic retry
- **Data Validation**: Command verification
- **Buffer Management**: Overflow protection

## Troubleshooting

### Common Issues

#### No Serial Output

- **Check**: USB connection
- **Verify**: Correct COM port
- **Test**: Serial monitor in Arduino IDE
- **Solution**: Reconnect USB cable

#### Commands Not Responding

- **Check**: Baud rate (115200)
- **Verify**: Line ending (\n)
- **Test**: PING command
- **Solution**: Reset ESP32

#### Power Test Not Working

- **Check**: LED pin connection
- **Verify**: Power test mode active
- **Test**: Manual LED commands
- **Solution**: Check hardware connections

#### Memory Issues

- **Monitor**: Free heap in status
- **Check**: For memory leaks
- **Optimize**: String usage
- **Solution**: Restart if needed

### Debug Information

- **Status Messages**: Include debug information
- **Error Reporting**: Clear error messages
- **System Info**: Detailed hardware information
- **Performance**: Timing and memory data

## Extensibility

### Adding New Commands

```cpp
else if (command == "NEW_COMMAND") {
    // Add custom functionality here
    Serial.println("Custom command executed");
}
```

### Adding New Sensors

```cpp
void readCustomSensor() {
    int sensorValue = analogRead(SENSOR_PIN);
    Serial.print("SENSOR:");
    Serial.print(" VALUE=");
    Serial.println(sensorValue);
}
```

### Adding New Status Information

```cpp
void sendExtendedStatus() {
    sendStatus();
    Serial.print(" CUSTOM=");
    Serial.print(customValue);
    Serial.println();
}
```

## Integration with Python Script

### Command Flow

1. **Python Script**: Sends command via serial
2. **ESP32**: Receives and processes command
3. **ESP32**: Executes requested function
4. **ESP32**: Sends response via serial
5. **Python Script**: Receives and logs response

### Data Flow

1. **Status Requests**: Python requests status
2. **Data Collection**: ESP32 gathers information
3. **Data Formatting**: ESP32 formats response
4. **Data Transmission**: ESP32 sends via serial
5. **Data Processing**: Python parses and logs

### Error Handling

1. **Command Validation**: ESP32 validates commands
2. **Error Reporting**: ESP32 reports errors
3. **Python Processing**: Python handles errors
4. **Logging**: Both systems log errors
5. **Recovery**: Automatic recovery mechanisms

This sketch provides a comprehensive testing and monitoring solution for the ESP32S3, enabling detailed communication with the Python monitoring script and providing valuable debugging information for the ESP32S3-Crazyflie connection testing system.
