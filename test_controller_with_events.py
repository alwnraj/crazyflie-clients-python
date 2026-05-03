#!/usr/bin/env python3
"""
Test Logitech F310 controller and display real-time events for a few seconds
"""

import struct
import os
import fcntl
import time
import select

JS_EVENT_FMT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)

JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

def quick_test(device_path="/dev/input/js1", duration=10):
    """Test controller for a few seconds and show events"""
    
    print("=" * 60)
    print("Logitech F310 Controller Real-Time Test")
    print("=" * 60)
    print(f"Testing device: {device_path}")
    print(f"Duration: {duration} seconds")
    print("\nMove the joysticks and press buttons to see events...")
    print("=" * 60 + "\n")
    
    try:
        js_file = open(device_path, "rb")
        fcntl.fcntl(js_file.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        
        axes = {}
        buttons = {}
        event_count = 0
        
        start_time = time.time()
        last_summary = start_time
        
        while (time.time() - start_time) < duration:
            readable, _, _ = select.select([js_file], [], [], 0.1)
            
            if readable:
                try:
                    event_data = js_file.read(JS_EVENT_SIZE)
                    
                    if len(event_data) == JS_EVENT_SIZE:
                        timestamp, value, event_type, number = struct.unpack(JS_EVENT_FMT, event_data)
                        event_type &= ~JS_EVENT_INIT
                        event_count += 1
                        
                        if event_type == JS_EVENT_BUTTON:
                            state = "PRESSED" if value else "RELEASED"
                            buttons[number] = value
                            print(f"[{time.time() - start_time:5.1f}s] Button {number:2d}: {state}")
                            
                        elif event_type == JS_EVENT_AXIS:
                            normalized = value / 32767.0
                            axes[number] = normalized
                            
                            # Map axis numbers to names (typical for F310)
                            axis_names = {
                                0: "Left Stick X ",
                                1: "Left Stick Y ",
                                2: "Right Stick X",
                                3: "Right Stick Y",
                                4: "L2/R2 Trigger",
                                5: "D-Pad      "
                            }
                            axis_name = axis_names.get(number, f"Axis {number}")
                            
                            if abs(normalized) > 0.05:
                                bar = "█" * int(abs(normalized) * 20)
                                direction = "←" if normalized < 0 else "→"
                                print(f"[{time.time() - start_time:5.1f}s] {axis_name}: {direction} {bar} ({normalized:6.2f})")
                                
                except OSError:
                    pass
            
            # Print summary every 2 seconds
            if time.time() - last_summary > 2.0:
                print(f"\n--- Status at {time.time() - start_time:.1f}s ---")
                print(f"Events received: {event_count}")
                print(f"Active buttons: {sum(1 for v in buttons.values() if v)}/{len(buttons)}")
                print(f"Axes with input: {len([v for v in axes.values() if abs(v) > 0.05])}/{len(axes)}")
                print()
                last_summary = time.time()
        
        js_file.close()
        
        print("\n" + "=" * 60)
        print("Test Complete!")
        print("=" * 60)
        print(f"Total events: {event_count}")
        print(f"Axes detected: {len(axes)}")
        print(f"Buttons detected: {len(buttons)}")
        
        if event_count > 0:
            print("\n✓ Controller is working and responding to input!")
        else:
            print("\n⚠ No events detected. Try moving sticks or pressing buttons.")
        
        print("=" * 60)
        return event_count > 0
        
    except FileNotFoundError:
        print(f"✗ Device {device_path} not found!")
        return False
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return False


if __name__ == "__main__":
    import sys
    
    device = "/dev/input/js1"  # F310 is usually js1
    duration = 10
    
    if len(sys.argv) > 1:
        device = sys.argv[1]
    if len(sys.argv) > 2:
        duration = int(sys.argv[2])
    
    print(f"\n🎮 Make sure your Logitech F310 is in DirectInput mode")
    print("   (The switch on the back should be set to 'D')\n")
    
    success = quick_test(device, duration)
    sys.exit(0 if success else 1)

