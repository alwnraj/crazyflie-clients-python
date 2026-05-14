#!/usr/bin/env python
"""
Monitor Crazyflie battery voltage/current and estimate electrical power.

Many Crazyflie firmwares expose battery voltage as pm.vbat but do not expose
measured battery current. This script discovers the connected drone's log TOC,
logs current when a known current variable is present, and otherwise reports
voltage only.
"""
import argparse
import logging
import statistics
import sys
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


DEFAULT_URI = "radio://0/80/2M"
CURRENT_CANDIDATES = (
    "pm.iBat",
    "pm.ibat",
    "pm.batteryCurrent",
    "pm.current",
)
VOLTAGE_VAR = "pm.vbat"


logging.basicConfig(level=logging.ERROR)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Read Crazyflie battery telemetry and print power draw when current is available."
    )
    parser.add_argument("uri", nargs="?", default=DEFAULT_URI, help=f"Crazyflie URI, default: {DEFAULT_URI}")
    parser.add_argument("-d", "--duration", type=float, default=10.0, help="seconds to monitor, default: 10")
    parser.add_argument("-p", "--period-ms", type=int, default=250, help="log period in ms, default: 250")
    parser.add_argument(
        "--current-var",
        help="override current log variable, for example pm.iBat if your firmware exposes it",
    )
    parser.add_argument(
        "--current-scale",
        type=float,
        default=None,
        help="multiply raw current by this to get amps, for example 0.001 for mA",
    )
    parser.add_argument("--list-pm", action="store_true", help="print available pm.* log variables and exit")
    return parser.parse_args()


def available_log_vars(cf, group=None):
    variables = []
    for toc_group, entries in cf.log.toc.toc.items():
        if group is not None and toc_group != group:
            continue
        for name in entries:
            variables.append(f"{toc_group}.{name}")
    return sorted(variables)


def choose_current_var(cf, override):
    if override:
        return override if cf.log.toc.get_element_by_complete_name(override) else None

    for variable in CURRENT_CANDIDATES:
        if cf.log.toc.get_element_by_complete_name(variable):
            return variable
    return None


def current_to_amps(raw_current, scale):
    if scale is not None:
        return raw_current * scale

    # Some firmwares report current in mA, some in A. Values above 20 are
    # almost certainly milliamps for a small Crazyflie-class battery.
    if abs(raw_current) > 20:
        return raw_current / 1000.0
    return raw_current


def print_summary(samples, current_var):
    print("\nSummary")
    print("-" * 60)
    voltages = [sample["voltage"] for sample in samples]
    print(f"Voltage: avg {statistics.mean(voltages):.3f} V, min {min(voltages):.3f} V, max {max(voltages):.3f} V")

    power_samples = [sample for sample in samples if sample.get("power") is not None]
    if power_samples:
        currents = [sample["current_amps"] for sample in power_samples]
        powers = [sample["power"] for sample in power_samples]
        print(f"Current ({current_var}): avg {statistics.mean(currents):.3f} A, max {max(currents):.3f} A")
        print(f"Power: avg {statistics.mean(powers):.3f} W, max {max(powers):.3f} W")
    else:
        print("Power: not available because this drone/firmware did not expose a current log variable.")
        print("Tip: run with --list-pm to see the exact pm.* telemetry variables available.")


def main():
    args = parse_args()
    samples = []

    print("=" * 60)
    print("Crazyflie Power Monitor")
    print("=" * 60)
    print(f"Connecting to {args.uri}...")

    cflib.crtp.init_drivers()

    with SyncCrazyflie(args.uri, cf=Crazyflie(rw_cache="./cache")) as scf:
        cf = scf.cf
        print("Connected.")

        pm_vars = available_log_vars(cf, "pm")
        if args.list_pm:
            print("\nAvailable pm.* log variables:")
            for variable in pm_vars:
                print(f"  {variable}")
            return 0

        if not cf.log.toc.get_element_by_complete_name(VOLTAGE_VAR):
            print(f"Error: required log variable {VOLTAGE_VAR} is not available.")
            return 1

        current_var = choose_current_var(cf, args.current_var)
        if args.current_var and current_var is None:
            print(f"Error: requested current variable {args.current_var} is not available.")
            print("Run with --list-pm to inspect available power-management variables.")
            return 1

        if current_var:
            print(f"Logging {VOLTAGE_VAR} and {current_var}.")
        else:
            print(f"Logging {VOLTAGE_VAR}. No known current variable found, so watts cannot be computed.")

        logconf = LogConfig(name="Power", period_in_ms=args.period_ms)
        logconf.add_variable(VOLTAGE_VAR)
        if current_var:
            logconf.add_variable(current_var)

        def on_data(timestamp, data, _logconf):
            voltage = float(data[VOLTAGE_VAR])
            sample = {"voltage": voltage}

            if current_var:
                raw_current = float(data[current_var])
                current_amps = current_to_amps(raw_current, args.current_scale)
                power = voltage * current_amps
                sample.update({"current_amps": current_amps, "power": power})
                print(
                    f"{timestamp:>8} ms  {voltage:5.3f} V  "
                    f"{current_amps:6.3f} A  {power:7.3f} W"
                )
            else:
                print(f"{timestamp:>8} ms  {voltage:5.3f} V")

            samples.append(sample)

        logconf.data_received_cb.add_callback(on_data)
        cf.log.add_config(logconf)

        print("\nPress Ctrl+C to stop early.\n")
        if current_var:
            print("timestamp     volts     amps     watts")
        else:
            print("timestamp     volts")
        print("-" * 60)

        try:
            logconf.start()
            time.sleep(max(args.duration, 0))
        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            logconf.stop()

    if samples:
        print_summary(samples, current_var)
    else:
        print("No samples received.")
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
