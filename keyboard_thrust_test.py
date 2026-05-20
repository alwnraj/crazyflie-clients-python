#!/usr/bin/env python3
"""
Terminal keyboard thrust test for Crazyflie.

This is a diagnostic script: it commands only zero roll, zero pitch, zero yaw
rate, and manually adjusted raw thrust. It does not use mocap, position hold,
or the high-level commander.
"""

import argparse
import curses
import logging
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


DEFAULT_URI = 'radio://0/80/2M'
MIN_THRUST = 0
MAX_THRUST = 65535
DEFAULT_MAX_COMMANDED_THRUST = 36000
DEFAULT_STEP = 500
DEFAULT_BIG_STEP = 2500
COMMAND_PERIOD = 0.02
BATTERY_PERIOD_MS = 500
LOW_BATTERY_VOLTAGE = 3.7
VERY_LOW_BATTERY_VOLTAGE = 3.5


class Telemetry:
    def __init__(self):
        self.battery_voltage = 0.0
        self.altitude = 0.0

    def battery_callback(self, timestamp, data, logconf):
        self.battery_voltage = data['pm.vbat']

    def altitude_callback(self, timestamp, data, logconf):
        self.altitude = data['stateEstimate.z']


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def send_zero_thrust(cf, count=10):
    for _ in range(count):
        cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
        time.sleep(COMMAND_PERIOD)
    cf.commander.send_stop_setpoint()


def draw(stdscr, thrust, max_commanded_thrust, telemetry, message):
    max_y, max_x = stdscr.getmaxyx()

    def add_line(y, x, text):
        if y >= max_y or x >= max_x:
            return
        # Avoid writing into the lower-right terminal cell; curses can raise
        # ERR there even when the text technically fits.
        available = max_x - x - 1
        if available <= 0:
            return
        stdscr.addstr(y, x, text[:available])

    thrust_pct = 100.0 * thrust / MAX_THRUST
    stdscr.erase()
    add_line(0, 0, "Crazyflie Keyboard Thrust Test")
    add_line(2, 0, "Controls:")
    add_line(3, 2, "UP / DOWN        thrust +/- small step")
    add_line(4, 2, "PAGEUP / PAGEDN  thrust +/- big step")
    add_line(5, 2, "SPACE            cut thrust to zero")
    add_line(6, 2, "q or ESC         cut, disarm, exit")
    add_line(8, 0, f"Thrust: {thrust:5d} / {MAX_THRUST} ({thrust_pct:5.1f}%)")
    add_line(9, 0, f"Commanded thrust cap: {max_commanded_thrust}")
    add_line(10, 0, f"Battery: {telemetry.battery_voltage:0.2f} V")
    add_line(11, 0, f"Estimator z: {telemetry.altitude:0.2f} m")
    add_line(13, 0, "Roll/Pitch/Yaw are fixed at zero.")
    add_line(14, 0, "Keep one hand ready to power off. Ctrl+C also cuts/disarms.")
    if message:
        add_line(16, 0, message)
    stdscr.refresh()


def run_keyboard_loop(stdscr, cf, telemetry, step, big_step, max_commanded_thrust):
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    thrust = 0
    message = "Starting at zero thrust."
    last_draw = 0.0

    while True:
        key = stdscr.getch()

        if key in (ord('q'), ord('Q'), 27):
            thrust = 0
            cf.commander.send_setpoint(0.0, 0.0, 0.0, 0)
            cf.commander.send_stop_setpoint()
            message = "Exit requested."
            draw(stdscr, thrust, max_commanded_thrust, telemetry, message)
            return thrust
        if key == ord(' '):
            thrust = 0
            message = "Thrust cut to zero."
        elif key == curses.KEY_UP:
            thrust = clamp(thrust + step, MIN_THRUST, max_commanded_thrust)
            message = f"Thrust increased by {step}."
        elif key == curses.KEY_DOWN:
            thrust = clamp(thrust - step, MIN_THRUST, max_commanded_thrust)
            message = f"Thrust decreased by {step}."
        elif key == curses.KEY_PPAGE:
            thrust = clamp(thrust + big_step, MIN_THRUST, max_commanded_thrust)
            message = f"Thrust increased by {big_step}."
        elif key == curses.KEY_NPAGE:
            thrust = clamp(thrust - big_step, MIN_THRUST, max_commanded_thrust)
            message = f"Thrust decreased by {big_step}."

        cf.commander.send_setpoint(0.0, 0.0, 0.0, thrust)

        now = time.time()
        if now - last_draw >= 0.1:
            draw(stdscr, thrust, max_commanded_thrust, telemetry, message)
            last_draw = now

        time.sleep(COMMAND_PERIOD)


def run(args):
    logging.basicConfig(level=logging.ERROR)
    cflib.crtp.init_drivers()

    telemetry = Telemetry()
    print("=" * 72)
    print("CRAZYFLIE KEYBOARD THRUST TEST")
    print("=" * 72)
    print(f"URI: {args.uri}")
    print(f"Small step: {args.step}, big step: {args.big_step}")
    print(f"Commanded thrust cap: {args.max_commanded_thrust}")
    print("This script sends zero roll/pitch/yawrate and raw thrust only.")
    print("Close cfclient first; only one process can own the radio.")
    print("=" * 72)
    input("Press ENTER to connect, or Ctrl+C to abort...")

    battery_logconf = LogConfig(name='Battery', period_in_ms=BATTERY_PERIOD_MS)
    battery_logconf.add_variable('pm.vbat', 'float')

    altitude_logconf = LogConfig(name='Altitude', period_in_ms=BATTERY_PERIOD_MS)
    altitude_logconf.add_variable('stateEstimate.z', 'float')

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        print("Connected.")

        cf.log.add_config(battery_logconf)
        battery_logconf.data_received_cb.add_callback(telemetry.battery_callback)
        battery_logconf.start()

        cf.log.add_config(altitude_logconf)
        altitude_logconf.data_received_cb.add_callback(telemetry.altitude_callback)
        altitude_logconf.start()
        time.sleep(0.8)

        print(f"Battery: {telemetry.battery_voltage:0.2f} V")
        if telemetry.battery_voltage < VERY_LOW_BATTERY_VOLTAGE:
            raise RuntimeError("Battery is very low. Do not fly.")
        elif telemetry.battery_voltage < LOW_BATTERY_VOLTAGE:
            print("WARNING: battery is low.")

        input("Press ENTER to arm and start at zero thrust, or Ctrl+C to abort...")

        last_thrust = 0
        try:
            cf.platform.send_arming_request(True)
            time.sleep(1.0)
            send_zero_thrust(cf, count=25)
            last_thrust = curses.wrapper(
                run_keyboard_loop,
                cf,
                telemetry,
                args.step,
                args.big_step,
                args.max_commanded_thrust,
            )
        finally:
            print("\nCutting thrust and disarming...")
            send_zero_thrust(cf, count=25)
            cf.platform.send_arming_request(False)
            altitude_logconf.stop()
            battery_logconf.stop()
            print(f"Stopped. Last commanded thrust was {last_thrust}.")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--uri', default=DEFAULT_URI)
    parser.add_argument('--step', type=int, default=DEFAULT_STEP)
    parser.add_argument('--big-step', type=int, default=DEFAULT_BIG_STEP)
    parser.add_argument(
        '--max-commanded-thrust',
        type=int,
        default=DEFAULT_MAX_COMMANDED_THRUST,
        help='Upper limit for keyboard-commanded raw thrust.',
    )
    args = parser.parse_args()
    if args.max_commanded_thrust < MIN_THRUST or args.max_commanded_thrust > MAX_THRUST:
        raise ValueError(
            f"--max-commanded-thrust must be between {MIN_THRUST} and {MAX_THRUST}"
        )
    return args


if __name__ == '__main__':
    run(parse_args())
