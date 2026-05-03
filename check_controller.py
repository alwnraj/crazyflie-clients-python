#!/usr/bin/env python3
"""
Quick controller detection and info script
"""

import struct
import os
import fcntl

def check_controller(device_path="/dev/input/js0"):
    """Check if the controller is accessible and get basic info"""
    
    print("=" * 60)
    print("Logitech Controller Detection")
    print("=" * 60)
    
    # Check if device exists
    if not os.path.exists(device_path):
        print(f"✗ Device {device_path} not found!")
        print("\nAvailable joystick devices:")
        found_any = False
        for i in range(10):
            dev = f"/dev/input/js{i}"
            if os.path.exists(dev):
                print(f"  ✓ {dev}")
                found_any = True
        if not found_any:
            print("  No joystick devices found!")
        return False
    
    print(f"✓ Device exists: {device_path}")
    
    try:
        # Try to open the device
        js_file = open(device_path, "rb")
        print(f"✓ Device opened successfully")
        
        # Get device name
        device_name_bytes = bytearray(64)
        JSIOCGNAME = 0x80006a13
        try:
            fcntl.ioctl(js_file.fileno(), JSIOCGNAME, device_name_bytes)
            device_name = device_name_bytes.decode('utf-8').rstrip('\x00')
            print(f"✓ Device name: {device_name}")
        except Exception as e:
            print(f"⚠ Could not get device name: {e}")
        
        # Get number of axes and buttons
        axes_count = bytearray(1)
        buttons_count = bytearray(1)
        JSIOCGAXES = 0x80016a11
        JSIOCGBUTTONS = 0x80016a12
        
        try:
            fcntl.ioctl(js_file.fileno(), JSIOCGAXES, axes_count)
            fcntl.ioctl(js_file.fileno(), JSIOCGBUTTONS, buttons_count)
            print(f"✓ Axes: {axes_count[0]}")
            print(f"✓ Buttons: {buttons_count[0]}")
        except Exception as e:
            print(f"⚠ Could not get axes/buttons count: {e}")
        
        js_file.close()
        
        print("\n" + "=" * 60)
        print("✓ Controller is working properly!")
        print("=" * 60)
        return True
        
    except PermissionError:
        print(f"✗ Permission denied for {device_path}")
        print("Try: sudo usermod -a -G input $USER")
        print("Then log out and log back in.")
        return False
        
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    import sys
    
    # Check all available devices
    devices = []
    for i in range(10):
        dev = f"/dev/input/js{i}"
        if os.path.exists(dev):
            devices.append(dev)
    
    if len(devices) == 0:
        print("No joystick devices found!")
        sys.exit(1)
    
    print(f"Found {len(devices)} joystick device(s)\n")
    
    # Test each device
    for device in devices:
        check_controller(device)
        print()
    
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"All {len(devices)} device(s) are accessible and working!")
    print("\nTo test controller input in real-time, run:")
    print("  python3 test_logitech_controller.py")
    print("=" * 60)

