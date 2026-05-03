#!/usr/bin/env python3
"""
Simple test script to verify Logitech controller input on Linux
Uses the native Linux joystick interface (/dev/input/js*)
"""

import struct
import os
import fcntl
import time
import select

# Linux joystick event format
JS_EVENT_FMT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)

# Event types
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

def test_controller(device_path="/dev/input/js0"):
    """Test the Logitech controller and display input events"""
    
    print("=" * 60)
    print("Logitech Controller Test")
    print("=" * 60)
    print(f"Opening device: {device_path}")
    
    try:
        # Open the joystick device
        js_file = open(device_path, "rb")
        
        # Set non-blocking mode
        fcntl.fcntl(js_file.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        
        # Get device name
        device_name_bytes = bytearray(64)
        JSIOCGNAME = 0x80006a13  # ioctl code for getting name
        try:
            fcntl.ioctl(js_file.fileno(), JSIOCGNAME, device_name_bytes)
            device_name = device_name_bytes.decode('utf-8').rstrip('\x00')
            print(f"Device name: {device_name}")
        except Exception as e:
            print(f"Could not get device name: {e}")
            device_name = "Unknown"
        
        # Get number of axes and buttons
        axes_count = bytearray(1)
        buttons_count = bytearray(1)
        JSIOCGAXES = 0x80016a11
        JSIOCGBUTTONS = 0x80016a12
        
        try:
            fcntl.ioctl(js_file.fileno(), JSIOCGAXES, axes_count)
            fcntl.ioctl(js_file.fileno(), JSIOCGBUTTONS, buttons_count)
            print(f"Axes: {axes_count[0]}, Buttons: {buttons_count[0]}")
        except Exception as e:
            print(f"Could not get axes/buttons count: {e}")
        
        print("\n" + "=" * 60)
        print("Controller is ready! Move sticks and press buttons...")
        print("Press Ctrl+C to exit")
        print("=" * 60 + "\n")
        
        axes = {}
        buttons = {}
        
        # Main event loop
        while True:
            # Use select to check if data is available
            readable, _, _ = select.select([js_file], [], [], 0.1)
            
            if readable:
                try:
                    event_data = js_file.read(JS_EVENT_SIZE)
                    
                    if len(event_data) == JS_EVENT_SIZE:
                        timestamp, value, event_type, number = struct.unpack(JS_EVENT_FMT, event_data)
                        
                        # Remove init flag
                        event_type &= ~JS_EVENT_INIT
                        
                        if event_type == JS_EVENT_BUTTON:
                            state = "PRESSED" if value else "RELEASED"
                            buttons[number] = value
                            print(f"Button {number:2d}: {state:8s} (value: {value})")
                            
                        elif event_type == JS_EVENT_AXIS:
                            # Normalize axis value (-32767 to 32767) to -1.0 to 1.0
                            normalized = value / 32767.0
                            axes[number] = normalized
                            
                            # Only print significant axis changes (dead zone)
                            if abs(normalized) > 0.05:
                                print(f"Axis   {number:2d}: {normalized:7.3f} (raw: {value:6d})")
                                
                except OSError:
                    # No more events available
                    time.sleep(0.01)
            else:
                # No events, just wait a bit
                time.sleep(0.01)
                
    except FileNotFoundError:
        print(f"\n✗ ERROR: Device {device_path} not found!")
        print("Available joystick devices:")
        for i in range(10):
            dev = f"/dev/input/js{i}"
            if os.path.exists(dev):
                print(f"  - {dev}")
        return False
        
    except PermissionError:
        print(f"\n✗ ERROR: Permission denied for {device_path}")
        print("Try running with sudo or add your user to the 'input' group:")
        print(f"  sudo usermod -a -G input $USER")
        print("Then log out and log back in.")
        return False
        
    except KeyboardInterrupt:
        print("\n\n" + "=" * 60)
        print("Test stopped by user")
        print("=" * 60)
        print(f"\nFinal state:")
        print(f"  Active buttons: {sum(1 for v in buttons.values() if v)}")
        print(f"  Active axes: {len(axes)}")
        print("\n✓ Controller test completed successfully!")
        
    finally:
        if 'js_file' in locals():
            js_file.close()
            print("Device closed.")
    
    return True


if __name__ == "__main__":
    import sys
    
    device = "/dev/input/js0"
    if len(sys.argv) > 1:
        device = sys.argv[1]
    
    # Check if we have multiple devices
    available = []
    for i in range(10):
        dev = f"/dev/input/js{i}"
        if os.path.exists(dev):
            available.append(dev)
    
    if len(available) > 1:
        print(f"Multiple joystick devices found: {', '.join(available)}")
        print(f"Testing: {device} (specify device as argument to test another)\n")
    
    success = test_controller(device)
    sys.exit(0 if success else 1)

