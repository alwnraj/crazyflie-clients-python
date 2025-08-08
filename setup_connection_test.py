#!/usr/bin/env python3
"""
Setup script for ESP32S3 - Crazyflie Connection Testing
This script helps set up the environment and provides instructions.
"""

import subprocess
import sys
import os

def check_python_version():
    """Check if Python version is compatible"""
    if sys.version_info < (3, 6):
        print("❌ Python 3.6 or higher is required")
        return False
    print(f"✅ Python {sys.version_info.major}.{sys.version_info.minor} detected")
    return True

def install_requirements():
    """Install required packages"""
    print("Installing required packages...")
    try:
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", "requirements.txt"])
        print("✅ Dependencies installed successfully")
        return True
    except subprocess.CalledProcessError as e:
        print(f"❌ Failed to install dependencies: {e}")
        return False

def check_serial_ports():
    """Check available serial ports"""
    try:
        import serial.tools.list_ports
        ports = list(serial.tools.list_ports.comports())
        
        print("\n📋 Available serial ports:")
        esp32_found = False
        for port in ports:
            print(f"  - {port.device}: {port.description}")
            if any(keyword in port.description.lower() for keyword in ['esp32', 'xiao', 'ch340', 'cp210']):
                esp32_found = True
        
        if not esp32_found:
            print("⚠️  No ESP32-related ports found")
            print("   Make sure your ESP32 is connected via USB")
        else:
            print("✅ ESP32 port detected")
            
        return True
    except ImportError:
        print("❌ pyserial not installed")
        return False

def print_instructions():
    """Print setup instructions"""
    print("\n" + "="*60)
    print("ESP32S3 - Crazyflie Connection Setup")
    print("="*60)
    print("\n📋 Setup Instructions:")
    print("1. Upload the ESP32 sketch:")
    print("   - Open Arduino IDE")
    print("   - Open esp32_test_sketch.ino")
    print("   - Select your ESP32S3 board")
    print("   - Upload the sketch")
    print("\n2. Connect the hardware:")
    print("   - Connect ESP32S3 3V3 to Crazyflie breakout VCC")
    print("   - Connect ESP32S3 GND to Crazyflie breakout GND")
    print("   - Connect Crazyflie via USB to computer")
    print("   - Connect ESP32S3 via USB to computer")
    print("\n3. Run the monitoring script:")
    print("   python esp32_crazyflie_connection_monitor.py")
    print("\n4. Test commands:")
    print("   - 'test' - Test power connection")
    print("   - 'power' - Send power test to ESP32")
    print("   - 'help' - Show available commands")
    print("\n🔧 Troubleshooting:")
    print("- If ESP32 not detected, check USB cable and drivers")
    print("- If Crazyflie not detected, check USB connection")
    print("- Check jumper wire connections")
    print("- Ensure both devices are powered on")
    print("="*60)

def main():
    """Main setup function"""
    print("Setting up ESP32S3 - Crazyflie Connection Test...")
    
    # Check Python version
    if not check_python_version():
        return
    
    # Install requirements
    if not install_requirements():
        return
    
    # Check serial ports
    check_serial_ports()
    
    # Print instructions
    print_instructions()
    
    print("\n✅ Setup complete! You can now run the monitoring script.")

if __name__ == "__main__":
    main()
