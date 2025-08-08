# ESP32S3 - Crazyflie Connection Monitor

This project helps you test and monitor the connection between a Xiao Sense ESP32S3 and Crazyflie using jumper cables through the breakout deck.

## Hardware Setup

### Required Components

- Xiao Sense ESP32S3
- Crazyflie 2.1
- Crazyflie Breakout Deck
- Jumper cables (male-to-male)
- USB cables for both devices

### Physical Connections

1. **Power Connection:**

   - Connect ESP32S3 3V3 pin to Crazyflie breakout VCC
   - Connect ESP32S3 GND pin to Crazyflie breakout GND

2. **USB Connections:**
   - Connect Crazyflie to computer via USB
   - Connect ESP32S3 to computer via USB

## Software Setup

### 1. Install Dependencies

```bash
python setup_connection_test.py
```

Or manually install:

```bash
pip install -r requirements.txt
```

### 2. Upload ESP32 Sketch

1. Open Arduino IDE
2. Open `esp32_test_sketch.ino`
3. Select your ESP32S3 board
4. Upload the sketch

### 3. Run the Monitoring Script

```bash
python esp32_crazyflie_connection_monitor.py
```

## Usage

### Interactive Commands

Once the monitoring script is running, you can use these commands:

- `help` - Show available commands
- `test` - Test power connection between devices
- `power` - Send power test command to ESP32
- `quit` - Exit the program

### What the Script Monitors

1. **Crazyflie Connection:**

   - USB connection status
   - Firmware version
   - Battery voltage
   - Communication health

2. **ESP32 Connection:**

   - Serial port detection
   - Communication response
   - Power draw monitoring
   - System status

3. **Power Connection:**
   - Voltage monitoring during power tests
   - Power draw detection
   - Connection stability

## Troubleshooting

### ESP32 Not Detected

- Check USB cable and drivers
- Try different USB port
- Verify ESP32 is powered on
- Check if ESP32 appears in Device Manager

### Crazyflie Not Detected

- Ensure Crazyflie is powered on
- Check USB connection
- Try different USB port
- Verify Crazyflie drivers are installed

### No Communication

- Check jumper wire connections
- Verify 3V3 and GND are properly connected
- Ensure both devices are powered
- Check for loose connections

### Power Issues

- Verify voltage levels
- Check for short circuits
- Ensure proper grounding
- Monitor battery voltage during tests

## File Descriptions

- `esp32_crazyflie_connection_monitor.py` - Main monitoring script
- `esp32_test_sketch.ino` - Arduino sketch for ESP32S3
- `setup_connection_test.py` - Setup and installation script
- `requirements.txt` - Python dependencies
- `connection_log.txt` - Generated log file (created when script runs)

## Expected Behavior

### Normal Operation

1. Script detects both devices
2. Establishes connections
3. Monitors status continuously
4. Responds to interactive commands
5. Logs all activity to file

### Power Test

1. ESP32 enters power test mode
2. LED blinks rapidly to simulate power draw
3. Battery voltage is monitored
4. Power consumption is logged

### Status Monitoring

- Real-time connection status
- Battery voltage tracking
- ESP32 system information
- Communication health checks

## Safety Notes

- Always disconnect power before making connections
- Double-check connections before powering on
- Monitor battery voltage during tests
- Don't exceed voltage/current limits
- Keep connections secure and stable

## Advanced Usage

### Custom ESP32 Commands

You can modify the ESP32 sketch to add custom commands:

```cpp
else if (command == "CUSTOM_CMD") {
    // Your custom code here
    Serial.println("Custom command executed");
}
```

### Extended Monitoring

The monitoring script can be extended to:

- Monitor additional sensors
- Log data to files
- Send alerts on connection issues
- Integrate with other systems

## Support

If you encounter issues:

1. Check the log file (`connection_log.txt`)
2. Verify all connections
3. Test each device individually
4. Check for driver issues
5. Ensure proper power supply

## License

This project is part of the Crazyflie ecosystem and follows the same licensing terms.
