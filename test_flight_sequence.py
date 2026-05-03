import logging
import time
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# URI to the Crazyflie to connect to (can be overridden via command line)
DEFAULT_URI = 'radio://0/80/2M'

# Flight parameters
THREE_FEET_IN_METERS = 0.2744  # 3 feet = 0.9144 meters
HOVER_THRUST = 8000  # Base hover thrust (adjust based on battery level)
TAKEOFF_DURATION = 3.0  # seconds
HOVER_DURATION = 1.0  # seconds
LAND_DURATION = 3.0  # seconds
RIGHT_MOVEMENT_DURATION = 1.0  # seconds

altitude_estimate = [0]
battery_voltage = [0]


def altitude_callback(timestamp, data, logconf):
    """Callback for altitude estimate"""
    altitude_estimate[0] = data['stateEstimate.z']
    # Print altitude periodically
    if timestamp % 1000 < 100:  # Print roughly once per second
        print(f"  Altitude: {altitude_estimate[0]:.2f}m")


def battery_callback(timestamp, data, logconf):
    """Callback for battery voltage"""
    battery_voltage[0] = data['pm.vbat']


def flight_sequence(cf):
    """
    Execute the flight sequence WITHOUT Flow deck:
    1. Take off to 3 feet (vertical only)
    2. Move right (will drift without position hold!)
    3. Controlled descent (land)
    
    WARNING: Without Flow deck, horizontal movement will drift!
    """
    print("=" * 60)
    print("Starting flight sequence (NO FLOW DECK MODE)")
    print("=" * 60)
    print("⚠ WARNING: Without Flow deck, the drone will drift!")
    print("   Vertical movement will work, but horizontal will be unstable.")
    print("=" * 60)
    
    # Adjust thrust based on battery
    thrust = HOVER_THRUST
    if battery_voltage[0] > 0:
        if battery_voltage[0] < 3.7:
            print(f"⚠ Battery is LOW ({battery_voltage[0]:.2f}V) - adding +14000 to thrust")
            thrust = HOVER_THRUST + 14000
        elif battery_voltage[0] < 3.9:
            print(f"⚠ Battery is moderate ({battery_voltage[0]:.2f}V) - adding +10000 to thrust")
            thrust = HOVER_THRUST + 10000
        else:
            print(f"✓ Battery is good ({battery_voltage[0]:.2f}V) - using HOVER_THRUST ({HOVER_THRUST})")
            thrust = HOVER_THRUST
    
    try:
        # Step 1: TAKEOFF - Gradual increase to hover thrust
        print(f"\nStep 1: Taking off to ~{THREE_FEET_IN_METERS:.2f} meters (3 feet)...")
        print("  Gradually increasing thrust...")
        
        # Ramp up thrust gradually
        for current_thrust in range(20000, thrust, 2000):
            for _ in range(5):
                cf.commander.send_setpoint(0, 0, 0, current_thrust)
                time.sleep(0.02)
        
        # Maintain hover thrust to reach altitude
        print(f"  Hovering at {thrust} thrust...")
        start_time = time.time()
        while (time.time() - start_time) < TAKEOFF_DURATION:
            cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(0.02)
        
        print(f"✓ Should be at altitude! Current: {altitude_estimate[0]:.2f}m")
        
        # Step 2: MOVE RIGHT - Apply roll command (will drift!)
        print(f"\nStep 2: Attempting to move right...")
        print("  ⚠ This will drift without position hold!")
        
        # Apply right roll (-10 degrees) while maintaining altitude
        roll_angle = -10.0  # Negative = right
        start_time = time.time()
        while (time.time() - start_time) < RIGHT_MOVEMENT_DURATION:
            cf.commander.send_setpoint(roll_angle, 0, 0, thrust)
            time.sleep(0.02)
        
        # Stop rolling, return to level
        print("  Returning to level flight...")
        for _ in range(50):
            cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(0.02)
        
        print("✓ Right movement complete (note: position drifted)")
        
        # Hover for observation
        print(f"\nHolding altitude for {HOVER_DURATION} seconds...")
        start_time = time.time()
        while (time.time() - start_time) < HOVER_DURATION:
            cf.commander.send_setpoint(0, 0, 0, thrust)
            time.sleep(0.02)
        
        # Step 3: LANDING - Gradual descent
        print(f"\nStep 3: Initiating controlled descent...")
        
        # Gradually reduce thrust
        for current_thrust in range(thrust, 20000, -2000):
            print(f"  Descending at thrust {current_thrust}...")
            for _ in range(15):
                cf.commander.send_setpoint(0, 0, 0, current_thrust)
                time.sleep(0.02)
        
        # Final gentle touchdown
        print("  Final descent...")
        for _ in range(50):
            cf.commander.send_setpoint(0, 0, 0, 20000)
            time.sleep(0.02)
        
        # Stop motors
        print("  Stopping motors...")
        cf.commander.send_stop_setpoint()
        
        print(f"✓ Landing complete! Final altitude: {altitude_estimate[0]:.2f}m")
        
    except Exception as e:
        print(f"\n✗ ERROR during flight: {e}")
        import traceback
        traceback.print_exc()
        # Emergency stop
        cf.commander.send_stop_setpoint()
        raise
    
    print("\n" + "=" * 60)
    print("Flight sequence finished!")
    print("=" * 60)


if __name__ == '__main__':
    # Get URI from command line or use default
    uri = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URI
    
    # Set up logging (ERROR level to reduce clutter)
    logging.basicConfig(level=logging.ERROR)
    
    print("=" * 60)
    print("Crazyflie Flight Test Sequence")
    print("(No Flow Deck Required)")
    print("=" * 60)
    print(f"URI: {uri}")
    print("\nIMPORTANT:")
    print("  - Charge battery to 4.0V+ before flight!")
    print("  - Place Crazyflie on a flat surface")
    print("  - Ensure clear flight space (2m radius)")
    print("  - Without Flow deck, drone WILL drift horizontally")
    print("=" * 60)
    
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    
    print("\nConnecting to Crazyflie...")
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("✓ Connected successfully!")
            
            # Check battery voltage
            print("\nChecking battery voltage...")
            battery_logconf = LogConfig(name='Battery', period_in_ms=500)
            battery_logconf.add_variable('pm.vbat', 'float')
            scf.cf.log.add_config(battery_logconf)
            battery_logconf.data_received_cb.add_callback(battery_callback)
            battery_logconf.start()
            time.sleep(1.0)
            
            print(f"Battery: {battery_voltage[0]:.2f}V")
            if battery_voltage[0] < 3.9:
                print("⚠ WARNING: Battery is low!")
                print("   For best results, charge to 4.0V+ before flying.")
                response = input("Continue anyway? (y/n): ")
                if response.lower() != 'y':
                    print("Aborting.")
                    battery_logconf.stop()
                    sys.exit(0)
            else:
                print("✓ Battery level is good")
            
            # Set up altitude logging
            print("\nSetting up altitude monitoring...")
            altitude_logconf = LogConfig(name='Altitude', period_in_ms=500)
            altitude_logconf.add_variable('stateEstimate.z', 'float')
            scf.cf.log.add_config(altitude_logconf)
            altitude_logconf.data_received_cb.add_callback(altitude_callback)
            altitude_logconf.start()
            time.sleep(0.5)
            
            print(f"Current altitude: {altitude_estimate[0]:.2f}m")
            
            # Arm the Crazyflie
            print("\nArming the Crazyflie...")
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)
            print("✓ Crazyflie armed")
            
            print("\nStarting flight sequence in 3 seconds...")
            print("⚠ BE READY - The motors will start spinning!")
            for i in range(3, 0, -1):
                print(f"  {i}...")
                time.sleep(1)
            print("\nSTARTING NOW!\n")
            
            try:
                flight_sequence(scf.cf)
            except KeyboardInterrupt:
                print("\n\n⚠ Flight sequence interrupted by user!")
                print("Emergency stop...")
                scf.cf.commander.send_stop_setpoint()
            except Exception as e:
                print(f"\n\n✗ ERROR during flight sequence: {e}")
                import traceback
                traceback.print_exc()
                print("\nEmergency stop...")
                scf.cf.commander.send_stop_setpoint()
            finally:
                # Stop logging and disarm
                print("\nCleaning up...")
                altitude_logconf.stop()
                battery_logconf.stop()
                time.sleep(0.5)
                scf.cf.platform.send_arming_request(False)
                print("✓ Crazyflie disarmed")
                print("\n" + "=" * 60)
                print("Test complete!")
                print("=" * 60)
                
    except Exception as e:
        print(f"\nFailed to connect: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)