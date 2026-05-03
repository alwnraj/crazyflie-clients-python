#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Simple motor test script - tests if motors respond to thrust commands
This does NOT require a Flow deck
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


def simple_thrust_test(scf):
    """
    Simple test that sends thrust commands to verify motors work
    WARNING: This will make the drone try to take off!
    Make sure it's on a flat surface or held securely
    """
    cf = scf.cf
    
    print("=" * 60)
    print("Simple Motor/Thrust Test")
    print("=" * 60)
    print("This will test if your motors respond to thrust commands.")
    print("The Crazyflie should lift off slightly.")
    print("\nMAKE SURE:")
    print("  - Crazyflie is on a flat, open surface")
    print("  - Area is clear of obstacles")
    print("  - You're ready to catch it if needed")
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
    
    print("\nSending thrust commands...")
    print("(Motors should start spinning NOW!)")
    
    # Send a series of increasing thrust commands
    # Thrust ranges from 0 to 65535 (16-bit)
    # Typical hover thrust is around 35000-45000 depending on battery
    
    try:
        # Start with low thrust
        print("\n1. Low thrust (20000) - motors should spin slowly")
        for _ in range(20):
            cf.commander.send_setpoint(0, 0, 0, 20000)
            time.sleep(0.1)
        
        # Medium thrust - should start to lift
        print("\n2. Medium thrust (35000) - should start lifting")
        for _ in range(20):
            cf.commander.send_setpoint(0, 0, 0, 35000)
            time.sleep(0.1)
        
        # Higher thrust - should lift off
        # print("\n3. Higher thrust (40000) - should lift off")
        # for _ in range(30):
        #     cf.commander.send_setpoint(0, 0, 0, 40000)
        #     time.sleep(0.1)
        
        # Back to medium thrust
        print("\n4. Reducing thrust (35000)")
        for _ in range(20):
            cf.commander.send_setpoint(0, 0, 0, 35000)
            time.sleep(0.1)
        
        # Gentle descent
        print("\n5. Descending (20000)")
        for _ in range(30):
            cf.commander.send_setpoint(0, 0, 0, 20000)
            time.sleep(0.1)
        
        # Stop
        print("\n6. Stopping motors")
        cf.commander.send_stop_setpoint()
        
        print("\n✓ Test complete!")
        
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
    print("Test finished!")
    print("=" * 60)


if __name__ == '__main__':
    # Get URI from command line or use default
    uri = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URI
    
    print("=" * 60)
    print("Crazyflie Simple Motor Test")
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
            
            simple_thrust_test(scf)
            
    except Exception as e:
        print(f"\nFailed to connect: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

