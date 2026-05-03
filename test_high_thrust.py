#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
High thrust test - uses higher thrust values to ensure liftoff
"""
import logging
import time
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# URI to the Crazyflie to connect to
DEFAULT_URI = 'radio://0/80/2M'

# Set up logging
logging.basicConfig(level=logging.ERROR)


def high_thrust_test(scf):
    """
    Test with progressively higher thrust values to find hover point
    WARNING: This WILL make the drone take off!
    """
    cf = scf.cf
    
    print("=" * 60)
    print("HIGH THRUST TEST - Finding Hover Point")
    print("=" * 60)
    print("This test will increase thrust until the drone lifts off.")
    print("BE READY TO CATCH IT!")
    print("\nMAKE SURE:")
    print("  - Battery is FULLY CHARGED (critical!)")
    print("  - Crazyflie is on a flat surface")
    print("  - Area is clear")
    print("  - You're ready to catch/turn off if needed")
    print("=" * 60)
    
    print("\nStarting test in 3 seconds...")
    for i in range(3, 0, -1):
        print(f"  {i}...")
        time.sleep(1)
    print("\nSTARTING NOW!")
    
    # Arm the drone
    print("\nArming...")
    try:
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        print("✓ Armed")
    except Exception as e:
        print(f"Warning: Could not arm: {e}")
    
    print("\nSending progressively higher thrust commands...")
    print("Watch for liftoff!\n")
    
    try:
        # Test range of thrust values
        # Max thrust is 65535, typical hover is 35000-50000 depending on battery
        thrust_levels = [
            (30000, "30000 - Low"),
            (35000, "35000 - Low-Medium"),
            (40000, "40000 - Medium"),
            (45000, "45000 - Medium-High"),
            (48000, "48000 - High"),
            (50000, "50000 - Higher"),
            (52000, "52000 - Very High"),
        ]
        
        for thrust, label in thrust_levels:
            print(f"Thrust: {label}")
            for i in range(25):  # Hold each level for 2.5 seconds
                cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(0.1)
            
            # Quick check - did it lift off?
            print(f"  -> Did it lift off? If yes, stopping test.")
            time.sleep(0.5)
        
        # If we get here, try maximum safe thrust
        print("\nTrying MAXIMUM thrust (55000)...")
        print("(If this doesn't work, battery is too low or there's a hardware issue)")
        for i in range(30):
            cf.commander.send_setpoint(0, 0, 0, 55000)
            time.sleep(0.1)
        
        # Gentle descent
        print("\nDescending...")
        for thrust in range(45000, 20000, -2000):
            for _ in range(10):
                cf.commander.send_setpoint(0, 0, 0, thrust)
                time.sleep(0.1)
        
        # Stop
        print("\nStopping motors")
        cf.commander.send_stop_setpoint()
        
        print("\n✓ Test complete!")
        
    except KeyboardInterrupt:
        print("\n\nTest interrupted!")
    except Exception as e:
        print(f"\nERROR: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Make sure motors stop
        print("\nSending stop command...")
        cf.commander.send_stop_setpoint()
        time.sleep(0.5)
        
        # Disarm
        print("Disarming...")
        try:
            cf.platform.send_arming_request(False)
            print("✓ Disarmed")
        except Exception as e:
            print(f"Warning: Could not disarm: {e}")
    
    print("\n" + "=" * 60)
    print("IMPORTANT: Check your battery level!")
    print("If the drone didn't lift off, the battery is likely too low.")
    print("A fully charged battery should show ~4.2V per cell.")
    print("=" * 60)


if __name__ == '__main__':
    # Get URI from command line or use default
    uri = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URI
    
    print("=" * 60)
    print("Crazyflie High Thrust Test")
    print("=" * 60)
    print(f"URI: {uri}")
    print("=" * 60)
    
    # Initialize the low-level drivers
    cflib.crtp.init_drivers()
    
    print("\nConnecting to Crazyflie...")
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("✓ Connected successfully!")
            time.sleep(1)
            
            high_thrust_test(scf)
            
    except Exception as e:
        print(f"\nFailed to connect: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

