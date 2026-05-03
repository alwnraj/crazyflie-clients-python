#!/usr/bin/env python3
"""
Crazyflie flight control with Logitech F310 gamepad
Integrates joystick input with the flight sequence
"""

import logging
import time
import sys
import struct
import os
import fcntl
import select
import threading

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# URI to the Crazyflie to connect to
DEFAULT_URI = 'radio://0/80/2M'

# Controller device
CONTROLLER_DEVICE = '/dev/input/js1'  # F310 is usually js1

# Linux joystick constants
JS_EVENT_FMT = "IhBB"
JS_EVENT_SIZE = struct.calcsize(JS_EVENT_FMT)
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80

# Flight parameters
MAX_THRUST = 60000
MIN_THRUST = 10001
HOVER_THRUST = 35000

altitude_estimate = [0]
battery_voltage = [0]


class ControllerReader:
    """Read input from Logitech F310 controller"""
    
    def __init__(self, device_path=CONTROLLER_DEVICE):
        self.device_path = device_path
        self.js_file = None
        self.running = False
        self.thread = None
        
        # Controller state
        self.axes = {}
        self.buttons = {}
        
        # Flight control values (normalized -1.0 to 1.0 or 0.0 to 1.0)
        self.roll = 0.0      # Axis 0: Left stick X
        self.pitch = 0.0     # Axis 1: Left stick Y
        self.yaw = 0.0       # Axis 3: Right stick X
        self.thrust = 0.0    # Axis 4: Right stick Y (converted to 0-1 range)
        
        # Button states
        self.alt_hold = False  # Button 5: RB
        self.emergency_stop = False  # Button 9: Start
        
        # Deadzone
        self.deadzone = 0.1
        
    def open(self):
        """Open the controller device"""
        if not os.path.exists(self.device_path):
            raise FileNotFoundError(f"Controller not found at {self.device_path}")
        
        self.js_file = open(self.device_path, "rb")
        fcntl.fcntl(self.js_file.fileno(), fcntl.F_SETFL, os.O_NONBLOCK)
        print(f"✓ Controller opened: {self.device_path}")
        
        # Get device info
        try:
            device_name_bytes = bytearray(64)
            JSIOCGNAME = 0x80006a13
            fcntl.ioctl(self.js_file.fileno(), JSIOCGNAME, device_name_bytes)
            device_name = device_name_bytes.decode('utf-8').rstrip('\x00')
            print(f"  Device: {device_name}")
        except:
            pass
    
    def close(self):
        """Close the controller device"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        if self.js_file:
            self.js_file.close()
            print("✓ Controller closed")
    
    def _apply_deadzone(self, value):
        """Apply deadzone to controller input"""
        if abs(value) < self.deadzone:
            return 0.0
        # Scale the remaining range to 0-1 or -1 to 1
        sign = 1 if value > 0 else -1
        return sign * (abs(value) - self.deadzone) / (1.0 - self.deadzone)
    
    def _read_loop(self):
        """Read controller events in a loop"""
        while self.running:
            try:
                readable, _, _ = select.select([self.js_file], [], [], 0.1)
                
                if readable:
                    event_data = self.js_file.read(JS_EVENT_SIZE)
                    
                    if len(event_data) == JS_EVENT_SIZE:
                        timestamp, value, event_type, number = struct.unpack(
                            JS_EVENT_FMT, event_data)
                        
                        # Remove init flag
                        event_type &= ~JS_EVENT_INIT
                        
                        if event_type == JS_EVENT_BUTTON:
                            self.buttons[number] = value
                            
                            # Map buttons to functions
                            if number == 5:  # RB - Alt hold
                                self.alt_hold = bool(value)
                                if value:
                                    print("  🔵 Alt Hold: ON")
                            elif number == 9:  # Start - Emergency stop
                                if value:
                                    self.emergency_stop = True
                                    print("  🔴 EMERGENCY STOP!")
                            
                        elif event_type == JS_EVENT_AXIS:
                            # Normalize axis value
                            normalized = value / 32767.0
                            self.axes[number] = normalized
                            
                            # Map axes to flight controls
                            if number == 0:  # Left stick X - Roll
                                self.roll = self._apply_deadzone(normalized)
                            elif number == 1:  # Left stick Y - Pitch
                                self.pitch = self._apply_deadzone(-normalized)  # Invert
                            elif number == 3:  # Right stick X - Yaw
                                self.yaw = self._apply_deadzone(normalized)
                            elif number == 4:  # Right stick Y - Thrust
                                # Convert from -1...1 to 0...1 (inverted)
                                self.thrust = (-normalized + 1.0) / 2.0
                                
            except OSError:
                pass
            except Exception as e:
                print(f"Controller read error: {e}")
                break
    
    def start(self):
        """Start reading controller input in a background thread"""
        self.running = True
        self.thread = threading.Thread(target=self._read_loop, daemon=True)
        self.thread.start()
        print("✓ Controller input thread started")
    
    def get_control_setpoint(self):
        """
        Get current control setpoint from controller
        Returns: (roll, pitch, yaw, thrust) in degrees/thrust units
        """
        # Convert normalized values to Crazyflie units
        # Roll/Pitch: -30 to +30 degrees
        # Yaw rate: -200 to +200 degrees/sec
        # Thrust: 10001 to 60000
        
        roll_deg = self.roll * 30.0
        pitch_deg = self.pitch * 30.0
        yaw_rate = self.yaw * 200.0
        thrust_value = MIN_THRUST + (self.thrust * (MAX_THRUST - MIN_THRUST))
        
        return roll_deg, pitch_deg, yaw_rate, int(thrust_value)


def altitude_callback(timestamp, data, logconf):
    """Callback for altitude estimate"""
    altitude_estimate[0] = data['stateEstimate.z']


def battery_callback(timestamp, data, logconf):
    """Callback for battery voltage"""
    battery_voltage[0] = data['pm.vbat']


def controller_flight(cf, controller):
    """
    Manual flight control using the controller
    """
    print("=" * 60)
    print("Manual Controller Flight Mode")
    print("=" * 60)
    print("Controls:")
    print("  Left Stick:   Roll & Pitch")
    print("  Right Stick:  Yaw & Thrust")
    print("  RB Button:    Altitude Hold (not implemented yet)")
    print("  Start Button: Emergency Stop")
    print("=" * 60)
    print("\n⚠ START with thrust at minimum (right stick down)!")
    print("⚠ Slowly increase thrust with right stick")
    print("\nPress Start button or Ctrl+C to stop\n")
    
    time.sleep(3)
    print("Starting in 3...")
    time.sleep(1)
    print("Starting in 2...")
    time.sleep(1)
    print("Starting in 1...")
    time.sleep(1)
    print("\n✓ CONTROLLER ACTIVE!\n")
    
    try:
        last_print = time.time()
        
        while not controller.emergency_stop:
            # Get control input from controller
            roll, pitch, yaw, thrust = controller.get_control_setpoint()
            
            # Send setpoint to Crazyflie
            cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            
            # Print status every second
            if time.time() - last_print > 1.0:
                print(f"Alt: {altitude_estimate[0]:.2f}m | "
                      f"Thrust: {thrust:5d} | "
                      f"R:{roll:5.1f}° P:{pitch:5.1f}° Y:{yaw:6.1f}°/s")
                last_print = time.time()
            
            time.sleep(0.02)  # 50Hz update rate
            
    except KeyboardInterrupt:
        print("\n\n⚠ Flight interrupted by keyboard!")
    finally:
        # Stop motors
        print("\nStopping motors...")
        cf.commander.send_stop_setpoint()


if __name__ == '__main__':
    # Get URI from command line or use default
    uri = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URI
    
    # Set up logging
    logging.basicConfig(level=logging.ERROR)
    
    print("=" * 60)
    print("Crazyflie Controller Flight Test")
    print("=" * 60)
    print(f"URI: {uri}")
    print(f"Controller: {CONTROLLER_DEVICE}")
    print("=" * 60)
    
    # Initialize controller
    controller = ControllerReader(CONTROLLER_DEVICE)
    
    try:
        controller.open()
        controller.start()
        
        # Give controller a moment to initialize
        time.sleep(0.5)
        
        print("\n✓ Controller ready!")
        print("  Move sticks to verify it's working...")
        time.sleep(2)
        
        # Initialize Crazyflie drivers
        cflib.crtp.init_drivers()
        
        print("\nConnecting to Crazyflie...")
        
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("✓ Connected to Crazyflie!")
            
            # Set up logging
            print("\nSetting up telemetry...")
            
            # Battery monitoring
            battery_logconf = LogConfig(name='Battery', period_in_ms=500)
            battery_logconf.add_variable('pm.vbat', 'float')
            scf.cf.log.add_config(battery_logconf)
            battery_logconf.data_received_cb.add_callback(battery_callback)
            battery_logconf.start()
            time.sleep(0.5)
            
            print(f"Battery: {battery_voltage[0]:.2f}V")
            if battery_voltage[0] < 3.7:
                print("⚠ WARNING: Battery is LOW!")
            
            # Altitude monitoring
            altitude_logconf = LogConfig(name='Altitude', period_in_ms=500)
            altitude_logconf.add_variable('stateEstimate.z', 'float')
            scf.cf.log.add_config(altitude_logconf)
            altitude_logconf.data_received_cb.add_callback(altitude_callback)
            altitude_logconf.start()
            time.sleep(0.5)
            
            # Arm the Crazyflie
            print("\nArming Crazyflie...")
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)
            print("✓ Armed")
            
            try:
                # Start manual flight
                controller_flight(scf.cf, controller)
                
            except Exception as e:
                print(f"\n✗ Error during flight: {e}")
                import traceback
                traceback.print_exc()
                scf.cf.commander.send_stop_setpoint()
                
            finally:
                # Cleanup
                print("\nCleaning up...")
                altitude_logconf.stop()
                battery_logconf.stop()
                time.sleep(0.5)
                scf.cf.platform.send_arming_request(False)
                print("✓ Disarmed")
                
    except FileNotFoundError:
        print(f"\n✗ Controller not found at {CONTROLLER_DEVICE}")
        print("Available devices:")
        for i in range(10):
            dev = f"/dev/input/js{i}"
            if os.path.exists(dev):
                print(f"  - {dev}")
        sys.exit(1)
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
        
    finally:
        controller.close()
        print("\n" + "=" * 60)
        print("Flight test complete!")
        print("=" * 60)

