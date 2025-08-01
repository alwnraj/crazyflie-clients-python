"""
This script shows a simple scripted flight path using the MotionCommander class.

Simple example that connects to the crazyflie at `URI` and runs a
sequence. Change the URI variable to your Crazyflie configuration.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    
    # Scan for available Crazyflies first
    print("Scanning for available Crazyflies...")
    available = cflib.crtp.scan_interfaces()
    
    if not available:
        print("No Crazyflies found! Please check:")
        print("1. Crazyflie is powered on")
        print("2. Crazyradio is connected")
        print("3. Crazyflie is within range")
        exit(1)
    
    print("Found Crazyflies:")
    for uri in available:
        print(f"  {uri}")
    
    # Use the first available Crazyflie or specify your own URI
    URI = available[0][0]  # Use first found Crazyflie
    print(f"Connecting to: {URI}")
    
    try:
        with SyncCrazyflie(URI) as scf:
            print("Connected!")
            # Add parameter checks here
            time.sleep(2)

            # Arm the Crazyflie
            scf.cf.platform.send_arming_request(True)
            time.sleep(1.0)

            # We take off when the commander is created
            with MotionCommander(scf) as mc:
                print('Taking off!')
                time.sleep(1)

                # There is a set of functions that move a specific distance
                # We can move in all directions
                print('Moving forward 0.5m')
                mc.forward(0.5)
                # Wait a bit
                time.sleep(1)

                print('Moving up 0.2m')
                mc.up(0.2)
                # Wait a bit
                time.sleep(1)

                print('Doing a 270deg circle');
                mc.circle_right(0.5, velocity=0.5, angle_degrees=270)

                print('Moving down 0.2m')
                mc.down(0.2)
                # Wait a bit
                time.sleep(1)

                print('Rolling left 0.2m at 0.6m/s')
                mc.left(0.2, velocity=0.6)
                # Wait a bit
                time.sleep(1)

                print('Moving forward 0.5m')
                mc.forward(0.5)

                # We land when the MotionCommander goes out of scope
                print('Landing!')
                
    except Exception as e:
        print(f"Connection failed: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure Crazyflie is powered on")
        print("2. Check that Crazyradio is properly connected")
        print("3. Try moving closer to the Crazyflie")
        print("4. Check for interference from other 2.4GHz devices")
        print("5. Try a different USB port for the Crazyradio")