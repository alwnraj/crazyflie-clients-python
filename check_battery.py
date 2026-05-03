#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Check Crazyflie battery voltage
"""
import logging
import time
import sys

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

# URI to the Crazyflie to connect to
DEFAULT_URI = 'radio://0/80/2M'

logging.basicConfig(level=logging.ERROR)

battery_voltage = [0]

def battery_callback(timestamp, data, logconf):
    """Callback for battery voltage"""
    battery_voltage[0] = data['pm.vbat']
    print(f"Battery voltage: {battery_voltage[0]:.2f}V")

if __name__ == '__main__':
    uri = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_URI
    
    print("=" * 60)
    print("Crazyflie Battery Check")
    print("=" * 60)
    
    cflib.crtp.init_drivers()
    
    print(f"Connecting to {uri}...")
    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            print("✓ Connected!")
            
            # Set up battery logging
            from cflib.crazyflie.log import LogConfig
            logconf = LogConfig(name='Battery', period_in_ms=1000)
            logconf.add_variable('pm.vbat', 'float')
            scf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(battery_callback)
            
            logconf.start()
            print("\nReading battery voltage for 3 seconds...")
            time.sleep(3)
            logconf.stop()
            
            print("\n" + "=" * 60)
            print(f"Final reading: {battery_voltage[0]:.2f}V")
            print("\nBattery Status:")
            if battery_voltage[0] >= 4.0:
                print("  ✓ GOOD - Battery is well charged")
            elif battery_voltage[0] >= 3.7:
                print("  ⚠ LOW - Battery is getting low, may affect flight")
            elif battery_voltage[0] >= 3.5:
                print("  ⚠ VERY LOW - Battery too low for reliable flight")
            else:
                print("  ✗ CRITICAL - Battery is dead or not connected properly")
            
            print("\nReference:")
            print("  4.2V = Fully charged")
            print("  3.7V = ~50% charge")
            print("  3.5V = Nearly empty")
            print("  <3.5V = Do not fly")
            print("=" * 60)
            
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()

