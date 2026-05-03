#!/usr/bin/env python3
"""
Live demonstration of Logitech F310 controller input
Shows real-time flight control values as you move the sticks
"""

import struct
import os
import fcntl
import select
import time
import sys

# Linux joystick constants
JS_EVENT_FMT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

CONTROLLER_DEVICE = '/dev/input/js1'

# Logitech F310 DirectInput axis mapping. If your controller reports a
# different layout, adjust these constants and keep test_flight_with_controller.py
# in sync.
ROLL_AXIS = 0
PITCH_AXIS = 1
YAW_AXIS = 2
THRUST_AXIS = 3


class FlightController:
    """Reads F310 and converts to flight control values"""
    
    def __init__(self):
        self.axes = [0.0] * 8
        self.buttons = [0] * 12
        self.deadzone = 0.1
        
    def apply_deadzone(self, value):
        if abs(value) < self.deadzone:
            return 0.0
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def get_flight_values(self):
        """Convert raw axes to flight control values"""
        # Left stick X -> Roll (-30 to +30 degrees)
        roll = self.apply_deadzone(self.axes[ROLL_AXIS]) * 30.0
        
        # Left stick Y -> Pitch (-30 to +30 degrees, inverted)
        pitch = self.apply_deadzone(-self.axes[PITCH_AXIS]) * 30.0
        
        # Right stick X -> Yaw (-200 to +200 deg/s)
        yaw = self.apply_deadzone(self.axes[YAW_AXIS]) * 200.0
        
        # Right stick Y -> Thrust (0 to 60000, inverted)
        thrust_norm = (-self.axes[THRUST_AXIS] + 1.0) / 2.0  # Convert -1..1 to 0..1
        thrust = int(thrust_norm * 60000)
        
        return roll, pitch, yaw, thrust
    
    def update_axis(self, number, value):
        """Update axis value (value is -32767 to 32767)"""
        if number < len(self.axes):
            self.axes[number] = value / 32767.0
    
    def update_button(self, number, value):
        """Update button value (0 or 1)"""
        if number < len(self.buttons):
            self.buttons[number] = value


def create_bar(value, width=20, min_val=-1.0, max_val=1.0):
    """Create a visual bar for a value"""
    normalized = (value - min_val) / (max_val - min_val)
    normalized = max(0.0, min(1.0, normalized))
    pos = min(width - 1, int(normalized * width))
    bar = ['-'] * width
    bar[pos] = '█'
    if pos > 0:
        for i in range(pos):
            bar[i] = '▓'
    return ''.join(bar)


def demo_controller():
    """Live demo of controller input"""
    
    print("\n" + "=" * 70)
    print(" Logitech F310 Controller - Live Flight Control Demo")
    print("=" * 70)
    controller_device = find_controller_device()
    print(f"Device: {controller_device}")
    print("\nMove the sticks and press buttons to see real-time values!")
    print("Press Ctrl+C to exit")
    print("=" * 70 + "\n")
    
    if not os.path.exists(controller_device):
        print(f"✗ Controller not found at {controller_device}")
        return False
    
    try:
        js_file = open(controller_device, "rb")
        fcntl.fcntl(js_file.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        
        controller = FlightController()
        last_update = time.time()
        update_interval = 0.1  # Update display every 100ms
        
        print("\033[?25l", end="")  # Hide cursor
        
        while True:
            readable, _, _ = select.select([js_file], [], [], 0.01)
            
            if readable:
                event_data = js_file.read(JS_EVENT_SIZE)
                
                if len(event_data) == JS_EVENT_SIZE:
                    timestamp, value, event_type, number = struct.unpack(
                        JS_EVENT_FMT, event_data)
                    
                    event_type &= ~JS_EVENT_INIT
                    
                    if event_type == JS_EVENT_BUTTON:
                        controller.update_button(number, value)
                        
                    elif event_type == JS_EVENT_AXIS:
                        controller.update_axis(number, value)
            
            # Update display periodically
            if time.time() - last_update > update_interval:
                roll, pitch, yaw, thrust = controller.get_flight_values()
                
                # Clear previous lines
                print("\033[2J\033[H", end="")
                
                print("=" * 70)
                print(" FLIGHT CONTROL VALUES (Real-time)")
                print("=" * 70)
                print()
                
                # Thrust (vertical)
                thrust_pct = (thrust / 60000) * 100
                print(f"  THRUST:  {thrust:5d}  [{thrust_pct:5.1f}%]  {create_bar(thrust_pct, 30, 0, 100)}")
                print()
                
                # Roll (left stick horizontal)
                print(f"  ROLL:    {roll:6.1f}°  {create_bar(roll, 40)}")
                print(f"           {'← LEFT':^20} {'CENTER':^20} {'RIGHT →':^20}")
                print()
                
                # Pitch (left stick vertical)
                print(f"  PITCH:   {pitch:6.1f}°  {create_bar(pitch, 40)}")
                print(f"           {'← BACK':^20} {'CENTER':^20} {'FORWARD →':^20}")
                print()
                
                # Yaw (right stick horizontal)
                print(f"  YAW:     {yaw:6.1f}°/s  {create_bar(yaw, 40, -200, 200)}")
                print(f"           {'← CCW':^20} {'CENTER':^20} {'CW →':^20}")
                print()
                
                print("=" * 70)
                print(" RAW AXES")
                print("=" * 70)
                for i, val in enumerate(controller.axes[:6]):
                    if abs(val) > 0.01:
                        print(f"  Axis {i}: {val:6.3f}  {create_bar(val, 30)}")
                
                print()
                print("=" * 70)
                print(" BUTTONS PRESSED")
                print("=" * 70)
                
                button_names = [
                    "X", "A", "B", "Y", "LB", "RB", 
                    "LT", "RT", "Back", "Start", "L3", "R3"
                ]
                
                pressed = []
                for i, state in enumerate(controller.buttons):
                    if state:
                        name = button_names[i] if i < len(button_names) else f"Button{i}"
                        pressed.append(name)
                
                if pressed:
                    print(f"  {', '.join(pressed)}")
                else:
                    print("  (none)")
                
                print()
                print("=" * 70)
                print(" Controls: Left Stick=Roll/Pitch | Right Stick=Yaw/Thrust")
                print(" Press Ctrl+C to exit")
                print("=" * 70)
                
                last_update = time.time()
    
    except KeyboardInterrupt:
        print("\n\n✓ Demo stopped")
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        print("\033[?25h", end="")  # Show cursor
        if 'js_file' in locals():
            js_file.close()
    
    return True


def find_controller_device(preferred=CONTROLLER_DEVICE):
    """Use the preferred joystick path if present, otherwise the first js device."""
    if os.path.exists(preferred):
        return preferred

    for i in range(10):
        dev = f"/dev/input/js{i}"
        if os.path.exists(dev):
            return dev

    return preferred


if __name__ == "__main__":
    print("\n🎮 Make sure your Logitech F310 is:")
    print("   - Set to DirectInput mode (switch on back set to 'D')")
    print("   - Plugged into a USB port")
    print()
    input("Press Enter to start the demo...")
    
    success = demo_controller()
    sys.exit(0 if success else 1)
