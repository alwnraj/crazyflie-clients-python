# Requirements File Documentation

## Overview

`requirements.txt` is a Python dependencies file that lists all the required packages for the ESP32S3-Crazyflie connection monitoring system.

## Purpose

This file ensures that all necessary Python packages are available for:

- Crazyflie communication and control
- Serial communication with ESP32
- Multi-threading for monitoring
- Logging and debugging capabilities

## Package Details

### cflib>=0.1.23

**Purpose**: Crazyflie library for communication and control
**Functionality**:

- Provides Crazyflie communication protocols
- Handles USB and radio connections
- Manages parameter reading/writing
- Supports firmware updates
- Enables flight control and telemetry

**Key Features**:

- Automatic device detection
- Connection management
- Parameter system
- Logging system
- Flight control commands

**Usage in Project**:

```python
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# Initialize drivers
cflib.crtp.init_drivers()

# Scan for devices
available = cflib.crtp.scan_interfaces()

# Connect to Crazyflie
cf = Crazyflie()
cf.open_link(uri)
```

### pyserial>=3.5

**Purpose**: Serial communication library for ESP32
**Functionality**:

- Cross-platform serial port access
- Configurable baud rates
- Timeout handling
- Port enumeration
- Device detection

**Key Features**:

- Platform-independent serial communication
- Automatic port detection
- Configurable communication parameters
- Error handling and recovery
- Thread-safe operations

**Usage in Project**:

```python
import serial
import serial.tools.list_ports

# List available ports
ports = list(serial.tools.list_ports.comports())

# Open serial connection
ser = serial.Serial(
    port='COM3',
    baudrate=115200,
    timeout=1,
    write_timeout=1
)

# Send/receive data
ser.write(b"PING\n")
response = ser.readline()
```

## Version Requirements

### Minimum Versions

- **cflib>=0.1.23**: Ensures compatibility with current Crazyflie firmware
- **pyserial>=3.5**: Provides stable serial communication features

### Version Selection Criteria

- **Stability**: Choose versions with proven reliability
- **Compatibility**: Ensure compatibility with target platforms
- **Features**: Include necessary functionality for the project
- **Security**: Avoid known security vulnerabilities

## Installation Methods

### Automatic Installation

```bash
# Using the setup script
python setup_connection_test.py

# Manual installation
pip install -r requirements.txt
```

### Individual Installation

```bash
# Install cflib
pip install cflib>=0.1.23

# Install pyserial
pip install pyserial>=3.5
```

### Virtual Environment Installation

```bash
# Create virtual environment
python -m venv venv

# Activate virtual environment
# Windows
venv\Scripts\activate
# Linux/Mac
source venv/bin/activate

# Install requirements
pip install -r requirements.txt
```

## Platform Compatibility

### Windows

- **cflib**: Full compatibility with Windows 10/11
- **pyserial**: Native Windows serial port support
- **Installation**: Standard pip installation works

### Linux

- **cflib**: Full compatibility with major distributions
- **pyserial**: Native Linux serial port support
- **Installation**: May require additional system packages

### macOS

- **cflib**: Full compatibility with macOS
- **pyserial**: Native macOS serial port support
- **Installation**: Standard pip installation works

## Dependencies of Dependencies

### cflib Dependencies

- **libusb**: USB communication (handled by cflib)
- **numpy**: Numerical operations (optional)
- **scipy**: Scientific computing (optional)

### pyserial Dependencies

- **No external dependencies**: Pure Python implementation
- **System serial drivers**: Provided by operating system

## Troubleshooting

### Installation Issues

#### cflib Installation Problems

```bash
# Common solutions
pip install --upgrade pip
pip install cflib --no-cache-dir
```

**Error**: "Microsoft Visual C++ 14.0 is required"

- **Solution**: Install Visual Studio Build Tools
- **Alternative**: Use pre-compiled wheels

**Error**: "Permission denied"

- **Solution**: Use `--user` flag or virtual environment
- **Alternative**: Run with administrator privileges

#### pyserial Installation Problems

```bash
# Common solutions
pip install pyserial --upgrade
pip install pyserial --force-reinstall
```

**Error**: "No module named 'serial'"

- **Solution**: Ensure pyserial is installed, not serial
- **Check**: `pip list | grep serial`

### Runtime Issues

#### cflib Runtime Problems

**Error**: "No Crazyflie devices found"

- **Check**: USB connection and drivers
- **Solution**: Install Crazyflie drivers

**Error**: "Connection timeout"

- **Check**: Device power and proximity
- **Solution**: Move closer to device

#### pyserial Runtime Problems

**Error**: "Access denied" on serial port

- **Check**: Port permissions and other applications
- **Solution**: Close other serial applications

**Error**: "Port not found"

- **Check**: Device connection and drivers
- **Solution**: Verify device is recognized by system

## Security Considerations

### Package Security

- **cflib**: Open source, community maintained
- **pyserial**: Open source, widely used
- **Updates**: Regular security updates available

### Network Security

- **Local communication**: No network exposure
- **USB communication**: Direct connection only
- **Serial communication**: Local device only

## Performance Considerations

### Memory Usage

- **cflib**: ~50MB typical usage
- **pyserial**: ~5MB typical usage
- **Total**: ~55MB for monitoring application

### CPU Usage

- **Monitoring**: Low CPU usage (~1-5%)
- **Communication**: Minimal overhead
- **Logging**: Asynchronous, non-blocking

## Development Considerations

### Adding New Dependencies

```txt
# requirements.txt
cflib>=0.1.23
pyserial>=3.5
new_package>=1.0.0  # Add new dependencies here
```

### Version Pinning

```txt
# For exact version control
cflib==0.1.23
pyserial==3.5.0
```

### Development Dependencies

```txt
# requirements-dev.txt
cflib>=0.1.23
pyserial>=3.5
pytest>=6.0.0
black>=21.0.0
flake8>=3.8.0
```

## Integration with Project

### Setup Script Integration

```python
# setup_connection_test.py
def install_requirements():
    subprocess.check_call([
        sys.executable, "-m", "pip", "install", "-r", "requirements.txt"
    ])
```

### Monitoring Script Integration

```python
# esp32_crazyflie_connection_monitor.py
import cflib.crtp
import serial
import serial.tools.list_ports
```

### Error Handling

```python
try:
    import cflib
except ImportError:
    print("cflib not installed. Run: pip install -r requirements.txt")

try:
    import serial
except ImportError:
    print("pyserial not installed. Run: pip install -r requirements.txt")
```

## Best Practices

### Version Management

- Use minimum version requirements
- Test with latest versions
- Document version compatibility
- Update requirements regularly

### Installation

- Use virtual environments
- Install in user space when possible
- Handle installation errors gracefully
- Provide clear error messages

### Maintenance

- Regular dependency updates
- Security vulnerability monitoring
- Compatibility testing
- Documentation updates

This requirements file ensures that all necessary dependencies are available for the ESP32S3-Crazyflie connection monitoring system, with proper version management and error handling.
