"""
This script shows a simple scripted flight path using the MotionCommander class.
Uses the exact URI that works with the GUI client.
"""
import logging
import time

import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander

# Use the exact URI that works with your GUI
URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)


if __name__ == '__main__':
    # Initialize the low-level drivers (don't list the debug drivers)
    cflib.crtp.init_drivers(enable_debug_driver=False)
    
    print(f"Connecting to Crazyflie at: {URI}")
    
    try:
        with SyncCrazyflie(URI) as scf:
            print("✓ Connected successfully!")
            
            # Check if we have a positioning deck
            try:
                flow_deck = scf.cf.param.get_value("deck.bcFlow2")
                if flow_deck == '1':
                    print("✓ Flow v2 deck detected")
                else:
                    print("⚠ No positioning deck detected - flight may be unstable")
            except:
                print("⚠ Could not check for positioning deck")

            # Arm the Crazyflie
            print("Arming Crazyflie...")
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
        print(f"✗ Connection failed: {e}")
        print("\nTroubleshooting tips:")
        print("1. Make sure Crazyflie is powered on")
        print("2. Check that Crazyradio is properly connected")
        print("3. Try moving closer to the Crazyflie")
        print("4. Check for interference from other 2.4GHz devices")
        print("5. Try a different USB port for the Crazyradio") 