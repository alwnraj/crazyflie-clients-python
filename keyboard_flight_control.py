#!/usr/bin/env python3
"""
Crazyflie flight control with keyboard input
Allows manual control of the drone using WASD + Arrow keys
"""

import logging
import time
import sys
import threading

try:
    from pynput import keyboard
except ImportError as e:
    error_msg = str(e)
    if 'X connection' in error_msg or 'DISPLAY' in error_msg or 'DisplayNameError' in error_msg:
        print("ERROR: pynput requires an X server connection!")
        print("=" * 60)
        print("This script needs access to your display/keyboard.")
        print("\nPossible solutions:")
        print("1. Make sure you're running this in a graphical terminal")
        print("2. If using SSH, enable X11 forwarding:")
        print("   ssh -X username@hostname")
        print("3. Set DISPLAY environment variable:")
        print("   export DISPLAY=:0")
        print("4. Check if X server is running:")
        print("   echo $DISPLAY")
        print("=" * 60)
    else:
        print("ERROR: pynput library not found!")
        print(f"Import error details: {e}")
        print("Install it with: pip install pynput")
        print("\nNote: On Linux, pynput may also require system packages:")
        print("  sudo apt-get install python3-xlib python3-dev")
        print("  sudo apt-get install libx11-dev libxtst-dev libxrandr-dev")
    sys.exit(1)
except Exception as e:
    error_msg = str(e)
    if 'X connection' in error_msg or 'DISPLAY' in error_msg or 'DisplayNameError' in error_msg:
        print("ERROR: pynput requires an X server connection!")
        print("=" * 60)
        print("This script needs access to your display/keyboard.")
        print("\nPossible solutions:")
        print("1. Make sure you're running this in a graphical terminal")
        print("2. If using SSH, enable X11 forwarding:")
        print("   ssh -X username@hostname")
        print("3. Set DISPLAY environment variable:")
        print("   export DISPLAY=:0")
        print("4. Check if X server is running:")
        print("   echo $DISPLAY")
        print("=" * 60)
    else:
        print(f"ERROR: Failed to import pynput: {e}")
        print("This might be a system dependency issue.")
        print("On Linux, try installing:")
        print("  sudo apt-get install python3-xlib python3-dev")
        print("  sudo apt-get install libx11-dev libxtst-dev libxrandr-dev")
    sys.exit(1)

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# ============================================================================
# CONFIGURABLE PARAMETERS - Adjust these values to customize behavior
# ============================================================================

# CONNECTION SETTINGS
# URI to the Crazyflie to connect to
# Format: 'radio://<channel>/<data_rate>/<address>'
# Example: 'radio://0/80/2M/E7E7E7E7E7'
DEFAULT_URI = 'radio://0/80/2M'  # <-- ADJUST: Change to match your Crazyflie radio settings

# FLIGHT PARAMETERS - Thrust limits
# These define the range of thrust values sent to the drone
MAX_THRUST = 60000      # <-- ADJUST: Maximum thrust value (PWM units)
MIN_THRUST = 100      # <-- ADJUST: Minimum thrust value (motors off)
HOVER_THRUST = 35000    # <-- ADJUST: Typical hover thrust (for reference, not used in code)

# CONTROL RATES - How fast control values change when holding keys
# Higher values = faster response, lower values = slower, more gradual control
ROLL_PITCH_RATE = 30.0      # <-- ADJUST: Roll/Pitch change rate (degrees per second)
YAW_RATE = 100.0            # <-- ADJUST: Yaw change rate (degrees per second)
THRUST_RATE = 5000.0       # <-- ADJUST: Thrust change rate (thrust units per second)

altitude_estimate = [0]
battery_voltage = [0]


class KeyboardReader:
    """Read input from keyboard and convert to flight control values"""
    
    def __init__(self):
        self.running = False
        self.listener = None
        
        # KEY MAPPINGS - Key states (True = pressed, False = released)
        # <-- ADJUST: To change key bindings, modify the keys checked in _on_press() and _on_release()
        self.keys_pressed = {
            'w': False,  # Pitch forward
            's': False,  # Pitch backward
            'a': False,  # Roll left
            'd': False,  # Roll right
            'left': False,  # Yaw left (CCW)
            'right': False,  # Yaw right (CW)
            'space': False,  # Thrust up
            'shift': False,  # Thrust down
        }
        
        # CURRENT CONTROL VALUES - Initial state
        # <-- ADJUST: Change initial values here (roll/pitch/yaw in degrees, thrust 0.0-1.0)
        self.roll = 0.0      # -30 to +30 degrees (initial roll angle)
        self.pitch = 0.0     # -30 to +30 degrees (initial pitch angle)
        self.yaw = 0.0       # -200 to +200 deg/s (initial yaw rate)
        # Start with minimum thrust (motors not spinning)
        # <-- ADJUST: Set to 0.0 for motors off, or higher (0.0-1.0) to start with motors spinning
        self.thrust = 0.0    # 0.0 to 1.0 (will be converted to MIN_THRUST to MAX_THRUST)
        
        # Control flags
        self.emergency_stop = False
        self.reset_requested = False
        
        # Last update time for rate limiting
        self.last_update = time.time()
        
        # Lock for thread-safe access
        self.lock = threading.Lock()
    
    def _on_press(self, key):
        """Handle key press events"""
        try:
            # Handle special keys
            if key == keyboard.Key.space:
                self.keys_pressed['space'] = True
            elif key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = True
            elif key == keyboard.Key.left:
                self.keys_pressed['left'] = True
            elif key == keyboard.Key.right:
                self.keys_pressed['right'] = True
            # <-- ADJUST: Emergency stop keys (currently Q and ESC)
            elif key == keyboard.Key.esc:
                self.emergency_stop = True
                print("\n  🔴 EMERGENCY STOP!")
            elif key == keyboard.KeyCode.from_char('q'):  # <-- ADJUST: Change 'q' to different key
                self.emergency_stop = True
                print("\n  🔴 EMERGENCY STOP!")
            # <-- ADJUST: Reset key (currently R)
            elif key == keyboard.KeyCode.from_char('r'):  # <-- ADJUST: Change 'r' to different key
                self.reset_requested = True
                print("\n  🔵 Controls reset to neutral")
            # Handle regular character keys
            elif hasattr(key, 'char') and key.char:
                char = key.char.lower()
                if char in self.keys_pressed:
                    self.keys_pressed[char] = True
        except AttributeError:
            pass
    
    def _on_release(self, key):
        """Handle key release events"""
        try:
            # Handle special keys
            if key == keyboard.Key.space:
                self.keys_pressed['space'] = False
            elif key == keyboard.Key.shift or key == keyboard.Key.shift_l or key == keyboard.Key.shift_r:
                self.keys_pressed['shift'] = False
            elif key == keyboard.Key.left:
                self.keys_pressed['left'] = False
            elif key == keyboard.Key.right:
                self.keys_pressed['right'] = False
            # Handle regular character keys
            elif hasattr(key, 'char') and key.char:
                char = key.char.lower()
                if char in self.keys_pressed:
                    self.keys_pressed[char] = False
        except AttributeError:
            pass
    
    def start(self):
        """Start listening for keyboard input"""
        self.running = True
        self.listener = keyboard.Listener(
            on_press=self._on_press,
            on_release=self._on_release
        )
        self.listener.start()
        print("✓ Keyboard input listener started")
    
    def close(self):
        """Stop listening for keyboard input"""
        self.running = False
        if self.listener:
            self.listener.stop()
        print("✓ Keyboard input listener stopped")
    
    def update_controls(self):
        """
        Update control values based on currently pressed keys
        Should be called regularly (e.g., at 50Hz)
        """
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time
        
        with self.lock:
            # Handle reset request
            if self.reset_requested:
                self.roll = 0.0
                self.pitch = 0.0
                self.yaw = 0.0
                # Don't reset thrust - keep current value
                self.reset_requested = False
            
            # Update roll (A/D keys)
            # <-- ADJUST: ROLL_PITCH_RATE controls how fast roll changes
            if self.keys_pressed['a']:
                self.roll -= ROLL_PITCH_RATE * dt
            elif self.keys_pressed['d']:
                self.roll += ROLL_PITCH_RATE * dt
            else:
                # Return to neutral if no key pressed
                # <-- ADJUST: 0.1 is the deadzone threshold, 0.9 is the decay rate (0.0-1.0)
                if abs(self.roll) > 0.1:  # <-- ADJUST: Deadzone threshold
                    self.roll *= 0.9  # <-- ADJUST: Decay rate (0.9 = 10% reduction per update)
                else:
                    self.roll = 0.0
            
            # Update pitch (W/S keys)
            # <-- ADJUST: ROLL_PITCH_RATE controls how fast pitch changes
            if self.keys_pressed['w']:
                self.pitch += ROLL_PITCH_RATE * dt
            elif self.keys_pressed['s']:
                self.pitch -= ROLL_PITCH_RATE * dt
            else:
                # Return to neutral if no key pressed
                # <-- ADJUST: 0.1 is the deadzone threshold, 0.9 is the decay rate
                if abs(self.pitch) > 0.1:  # <-- ADJUST: Deadzone threshold
                    self.pitch *= 0.9  # <-- ADJUST: Decay rate (0.9 = 10% reduction per update)
                else:
                    self.pitch = 0.0
            
            # Update yaw (Left/Right arrow keys)
            # <-- ADJUST: YAW_RATE controls how fast yaw changes
            if self.keys_pressed['left']:
                self.yaw -= YAW_RATE * dt
            elif self.keys_pressed['right']:
                self.yaw += YAW_RATE * dt
            else:
                # Return to neutral if no key pressed
                # <-- ADJUST: 5.0 is the deadzone threshold, 0.9 is the decay rate
                if abs(self.yaw) > 5.0:  # <-- ADJUST: Deadzone threshold (degrees/second)
                    self.yaw *= 0.9  # <-- ADJUST: Decay rate (0.9 = 10% reduction per update)
                else:
                    self.yaw = 0.0
            
            # Update thrust (Space/Shift keys)
            # <-- ADJUST: THRUST_RATE controls how fast thrust changes
            if self.keys_pressed['space']:
                self.thrust += (THRUST_RATE / (MAX_THRUST - MIN_THRUST)) * dt
            elif self.keys_pressed['shift']:
                self.thrust -= (THRUST_RATE / (MAX_THRUST - MIN_THRUST)) * dt
            # Thrust doesn't auto-return to neutral - stays at current value
            
            # CONTROL LIMITS - Maximum and minimum values for each control
            # <-- ADJUST: Change these limits to restrict control ranges
            self.roll = max(-30.0, min(30.0, self.roll))    # <-- ADJUST: Roll limit (±degrees)
            self.pitch = max(-30.0, min(30.0, self.pitch))  # <-- ADJUST: Pitch limit (±degrees)
            self.yaw = max(-200.0, min(200.0, self.yaw))    # <-- ADJUST: Yaw limit (±deg/s)
            # Ensure thrust can go all the way to minimum (0.0) when shift is held
            self.thrust = max(0.0, min(1.0, self.thrust))   # <-- ADJUST: Thrust limit (0.0-1.0)
    
    def get_control_setpoint(self):
        """
        Get current control setpoint from keyboard
        Returns: (roll, pitch, yaw, thrust) in degrees/thrust units
        """
        with self.lock:
            # Convert thrust from 0-1 range to MIN_THRUST-MAX_THRUST
            thrust_value = MIN_THRUST + (self.thrust * (MAX_THRUST - MIN_THRUST))
            
            return self.roll, self.pitch, self.yaw, int(thrust_value)


def altitude_callback(timestamp, data, logconf):
    """Callback for altitude estimate"""
    altitude_estimate[0] = data['stateEstimate.z']


def battery_callback(timestamp, data, logconf):
    """Callback for battery voltage"""
    battery_voltage[0] = data['pm.vbat']


def keyboard_flight(cf, keyboard_reader):
    """
    Manual flight control using keyboard input
    """
    print("=" * 60)
    print("Manual Keyboard Flight Mode")
    print("=" * 60)
    print("Controls:")
    print("  W/S:          Pitch forward/backward")
    print("  A/D:          Roll left/right")
    print("  ←/→ Arrow:    Yaw left/right")
    print("  Space/Shift:  Thrust up/down")
    print("  R:            Reset controls to neutral")
    print("  Q or ESC:     Emergency stop")
    print("=" * 60)
    print("\n⚠ Motors start at minimum thrust (not spinning)")
    print("⚠ Use Space to increase thrust, Shift to decrease")
    print("⚠ Hold Shift to reduce thrust to minimum (stop motors)")
    print("\nPress Q/ESC or Ctrl+C to stop\n")
    
    # COUNTDOWN TIMING - Delay before starting flight controls
    # <-- ADJUST: Change these delays to adjust countdown timing
    time.sleep(3)  # <-- ADJUST: Initial delay (seconds)
    print("Starting in 3...")
    time.sleep(1)  # <-- ADJUST: Countdown interval (seconds)
    print("Starting in 2...")
    time.sleep(1)  # <-- ADJUST: Countdown interval (seconds)
    print("Starting in 1...")
    time.sleep(1)  # <-- ADJUST: Countdown interval (seconds)
    print("\n✓ KEYBOARD CONTROLS ACTIVE!\n")
    print("Focus this window and use WASD + Arrow keys to control the drone")
    print("=" * 60 + "\n")
    
    try:
        last_print = time.time()
        
        while not keyboard_reader.emergency_stop:
            # Update control values based on key states
            keyboard_reader.update_controls()
            
            # Get control input from keyboard
            roll, pitch, yaw, thrust = keyboard_reader.get_control_setpoint()
            
            # Send setpoint to Crazyflie
            cf.commander.send_setpoint(roll, pitch, yaw, thrust)
            
            # STATUS DISPLAY - Print status periodically
            # <-- ADJUST: 1.0 is how often status is printed (seconds)
            if time.time() - last_print > 1.0:  # <-- ADJUST: Status print interval (seconds)
                thrust_pct = ((thrust - MIN_THRUST) / (MAX_THRUST - MIN_THRUST)) * 100
                print(f"Alt: {altitude_estimate[0]:.2f}m | "
                      f"Thrust: {thrust:5d} ({thrust_pct:5.1f}%) | "
                      f"R:{roll:5.1f}° P:{pitch:5.1f}° Y:{yaw:6.1f}°/s")
                last_print = time.time()
            
            # CONTROL UPDATE RATE - How often commands are sent to the drone
            # <-- ADJUST: 0.02 = 50Hz (20ms), lower = faster updates, higher = slower
            time.sleep(0.02)  # <-- ADJUST: Update rate (0.02 = 50Hz, 0.01 = 100Hz, 0.05 = 20Hz)
            
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
    print("Crazyflie Keyboard Flight Control")
    print("=" * 60)
    print(f"URI: {uri}")
    print("=" * 60)
    
    # Initialize keyboard reader
    keyboard_reader = KeyboardReader()
    
    try:
        keyboard_reader.start()
        
        # INITIALIZATION DELAYS
        # <-- ADJUST: Delay after starting keyboard listener (seconds)
        time.sleep(0.5)  # <-- ADJUST: Keyboard initialization delay
        
        print("\n✓ Keyboard ready!")
        print("  Make sure this window has focus for keyboard input")
        # <-- ADJUST: Delay before connecting to drone (seconds)
        time.sleep(1)  # <-- ADJUST: Pre-connection delay
        
        # Initialize Crazyflie drivers
        cflib.crtp.init_drivers()
        
        print("\nConnecting to Crazyflie...")
        
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("✓ Connected to Crazyflie!")
            
            # Set up logging
            print("\nSetting up telemetry...")
            
            # BATTERY MONITORING
            # <-- ADJUST: Battery log period (milliseconds)
            battery_logconf = LogConfig(name='Battery', period_in_ms=500)  # <-- ADJUST: Log period (ms)
            battery_logconf.add_variable('pm.vbat', 'float')
            scf.cf.log.add_config(battery_logconf)
            battery_logconf.data_received_cb.add_callback(battery_callback)
            battery_logconf.start()
            time.sleep(0.5)  # <-- ADJUST: Wait time after starting battery log
            
            print(f"Battery: {battery_voltage[0]:.2f}V")
            # <-- ADJUST: Battery voltage thresholds (volts)
            if battery_voltage[0] < 3.7:  # <-- ADJUST: Low battery threshold
                print("⚠ WARNING: Battery is LOW!")
            elif battery_voltage[0] < 3.5:  # <-- ADJUST: Very low battery threshold
                print("⚠⚠ WARNING: Battery is VERY LOW! Consider charging before flight.")
            
            # ALTITUDE MONITORING
            # <-- ADJUST: Altitude log period (milliseconds)
            altitude_logconf = LogConfig(name='Altitude', period_in_ms=500)  # <-- ADJUST: Log period (ms)
            altitude_logconf.add_variable('stateEstimate.z', 'float')
            scf.cf.log.add_config(altitude_logconf)
            altitude_logconf.data_received_cb.add_callback(altitude_callback)
            altitude_logconf.start()
            time.sleep(0.5)  # <-- ADJUST: Wait time after starting altitude log
            
            # ARMING DELAY
            # <-- ADJUST: Arming delay (seconds)
            print("\nArming Crazyflie...")
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)  # <-- ADJUST: Arming delay (seconds)
            print("✓ Armed")
            
            try:
                # Start manual flight
                keyboard_flight(scf.cf, keyboard_reader)
                
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
                
    except Exception as e:
        print(f"\n✗ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
        
    finally:
        keyboard_reader.close()
        print("\n" + "=" * 60)
        print("Flight control complete!")
        print("=" * 60)

