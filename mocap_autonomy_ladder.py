#!/usr/bin/env python3
"""
Canonical AIMSLab mocap autonomy ladder for Crazyflie indoor flight.

This script is the main path toward autonomous indoor figure-8 flight:

    OptiTrack/Motive -> VRPN -> cf.extpos -> Kalman estimator -> HLC position commands

Modes are intentionally staged. Run validate first, then hover, then steps, then
circle, and only then figure8.
"""

import argparse
import csv
import math
import select
import sys
import termios
import time
import tty
from dataclasses import dataclass
from pathlib import Path
from threading import Lock
from threading import Thread


DEFAULT_URI = 'radio://0/80/2M'
DEFAULT_HOST_NAME = '192.168.1.42:3883'
DEFAULT_RIGID_BODY_NAME = 'crazyflie_21'
DEFAULT_OUTPUT_DIR = 'flight_logs'

DEFAULT_X_MIN = -0.10
DEFAULT_X_MAX = 2.10
DEFAULT_Y_MIN = -2.60
DEFAULT_Y_MAX = 0.20
DEFAULT_Z_MIN = 0.00
DEFAULT_Z_MAX = 2.00

LOG_PERIOD_MS = 100
COMMAND_PERIOD = 0.10
MOCAP_TIMEOUT = 8.0
POSE_STABILITY_SECONDS = 2.0
POSE_STABILITY_MAX_RANGE = 0.05
ESTIMATE_MATCH_SECONDS = 2.0
ESTIMATE_MAX_AGE = 0.50
LOW_BATTERY_VOLTAGE = 3.70


def load_runtime_modules():
    import cflib.crtp
    import motioncapture
    from cflib.crazyflie import Crazyflie
    from cflib.crazyflie.log import LogConfig
    from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
    from cflib.utils.reset_estimator import reset_estimator

    return {
        'cflib_crtp': cflib.crtp,
        'motioncapture': motioncapture,
        'Crazyflie': Crazyflie,
        'LogConfig': LogConfig,
        'SyncCrazyflie': SyncCrazyflie,
        'reset_estimator': reset_estimator,
    }


@dataclass(frozen=True)
class Bounds:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float
    margin: float

    def check(self, position):
        x, y, z = position
        if x < self.x_min + self.margin:
            return False, f"x={x:.3f} is below x_min+margin"
        if x > self.x_max - self.margin:
            return False, f"x={x:.3f} is above x_max-margin"
        if y < self.y_min + self.margin:
            return False, f"y={y:.3f} is below y_min+margin"
        if y > self.y_max - self.margin:
            return False, f"y={y:.3f} is above y_max-margin"
        if z < self.z_min:
            return False, f"z={z:.3f} is below z_min"
        if z > self.z_max:
            return False, f"z={z:.3f} is above z_max"
        return True, "inside bounds"

    def contains_path(self, points):
        for point in points:
            safe, reason = self.check(point)
            if not safe:
                return False, reason
        return True, "inside bounds"


class GuardTrip(RuntimeError):
    def __init__(self, reason, immediate_stop=False):
        super().__init__(reason)
        self.reason = reason
        self.immediate_stop = immediate_stop


class OperatorAbort(RuntimeError):
    pass


class MocapState:
    def __init__(self):
        self._lock = Lock()
        self.position = None
        self.quat = None
        self.last_update = 0.0
        self.frame_count = 0
        self.found_body = False

    def update(self, position, quat):
        with self._lock:
            self.position = tuple(position)
            self.quat = quat
            self.last_update = time.time()
            self.frame_count += 1
            self.found_body = True

    def snapshot(self):
        with self._lock:
            return (
                self.position,
                self.quat,
                self.last_update,
                self.frame_count,
                self.found_body,
            )


class EstimateState:
    def __init__(self):
        self._lock = Lock()
        self.position = None
        self.battery_voltage = 0.0
        self.last_update = 0.0

    def update(self, x, y, z, battery_voltage):
        with self._lock:
            self.position = (x, y, z)
            self.battery_voltage = battery_voltage
            self.last_update = time.time()

    def snapshot(self):
        with self._lock:
            return self.position, self.battery_voltage, self.last_update


class MocapReader(Thread):
    def __init__(self, motioncapture_module, host_name, body_name, pose_mode, state):
        Thread.__init__(self)
        self.daemon = True
        self.motioncapture = motioncapture_module
        self.host_name = host_name
        self.body_name = body_name
        self.pose_mode = pose_mode
        self.state = state
        self.on_pose = None
        self.error = None
        self._stay_open = True

    def close(self):
        self._stay_open = False

    def run(self):
        try:
            mc = self.motioncapture.connect('vrpn', {'hostname': self.host_name})
            print(f"[INFO] Mocap connected, looking for '{self.body_name}'")
            announced = False
            while self._stay_open:
                mc.waitForNextFrame()
                for name, obj in mc.rigidBodies.items():
                    if name != self.body_name:
                        continue
                    if not announced:
                        print(f"[INFO] Found and tracking rigid body: {name}")
                        announced = True
                    pos = obj.position
                    quat = obj.rotation
                    self.state.update((pos[0], pos[1], pos[2]), quat)
                    if self.on_pose is not None:
                        self.on_pose(pos[0], pos[1], pos[2], quat)
        except Exception as exc:
            self.error = exc


class CsvLogger:
    FIELDNAMES = [
        'wall_time_s',
        'elapsed_s',
        'mode',
        'phase',
        'command',
        'stop_reason',
        'target_x',
        'target_y',
        'target_z',
        'target_yaw_deg',
        'mocap_x',
        'mocap_y',
        'mocap_z',
        'mocap_qx',
        'mocap_qy',
        'mocap_qz',
        'mocap_qw',
        'mocap_age_s',
        'mocap_frame_count',
        'estimate_x',
        'estimate_y',
        'estimate_z',
        'estimate_age_s',
        'estimate_error_m',
        'battery_v',
        'height_above_start_m',
        'radius_from_start_m',
        'target_error_m',
        'guard_ok',
    ]

    def __init__(self, output_path, mode):
        self.output_path = output_path
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.csv_file = self.output_path.open('w', newline='')
        self.writer = csv.DictWriter(self.csv_file, fieldnames=self.FIELDNAMES)
        self.writer.writeheader()
        self.mode = mode
        self.started_at = time.time()

    def write(self, row):
        now = time.time()
        full_row = {
            'wall_time_s': now,
            'elapsed_s': now - self.started_at,
            'mode': self.mode,
        }
        full_row.update(row)
        self.writer.writerow(full_row)
        self.csv_file.flush()

    def close(self):
        self.csv_file.close()


class RunStats:
    def __init__(self):
        self.max_estimate_error = 0.0
        self.max_height_above_start = 0.0
        self.max_radius_from_start = 0.0
        self.max_target_error = 0.0
        self.min_battery = None
        self.rows = 0

    def update(self, estimate_error, height, radius, target_error, battery):
        self.rows += 1
        if is_finite(estimate_error):
            self.max_estimate_error = max(self.max_estimate_error, estimate_error)
        if is_finite(height):
            self.max_height_above_start = max(self.max_height_above_start, height)
        if is_finite(radius):
            self.max_radius_from_start = max(self.max_radius_from_start, radius)
        if is_finite(target_error):
            self.max_target_error = max(self.max_target_error, target_error)
        if is_finite(battery):
            if self.min_battery is None:
                self.min_battery = battery
            else:
                self.min_battery = min(self.min_battery, battery)


class TerminalKeyReader:
    def __init__(self):
        self.enabled = sys.stdin.isatty()
        self._old_settings = None

    def __enter__(self):
        if self.enabled:
            self._old_settings = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin.fileno())
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.enabled and self._old_settings is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._old_settings)

    def poll_stop(self):
        if not self.enabled:
            return None
        readable, _, _ = select.select([sys.stdin], [], [], 0)
        if not readable:
            return None
        key = sys.stdin.read(1)
        if key in ('q', 'Q', ' ', '\x1b'):
            return key
        return None


def is_finite(value):
    return value is not None and math.isfinite(value)


def distance_2d(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def distance_3d(a, b):
    return math.sqrt(
        (a[0] - b[0]) ** 2
        + (a[1] - b[1]) ** 2
        + (a[2] - b[2]) ** 2
    )


def pose_age(mocap_state):
    _, _, last_update, _, _ = mocap_state.snapshot()
    if last_update == 0.0:
        return float('inf')
    return time.time() - last_update


def estimate_age(estimate_state):
    _, _, last_update = estimate_state.snapshot()
    if last_update == 0.0:
        return float('inf')
    return time.time() - last_update


def make_output_path(output, mode):
    if output:
        return Path(output)
    timestamp = time.strftime('%Y%m%d-%H%M%S')
    return Path(DEFAULT_OUTPUT_DIR) / f"mocap-autonomy-{mode}-{timestamp}.csv"


def make_bounds(args):
    return Bounds(
        x_min=args.x_min,
        x_max=args.x_max,
        y_min=args.y_min,
        y_max=args.y_max,
        z_min=args.z_min,
        z_max=args.z_max,
        margin=args.boundary_margin,
    )


def figure8_target(center, radius_x, radius_y, period_s, elapsed_s, z):
    phase = 2.0 * math.pi * elapsed_s / period_s
    return (
        center[0] + radius_x * math.sin(phase),
        center[1] + radius_y * math.sin(phase) * math.cos(phase),
        z,
    )


def circle_target(center, radius, period_s, elapsed_s, z):
    phase = 2.0 * math.pi * elapsed_s / period_s
    return (
        center[0] + radius * math.cos(phase),
        center[1] + radius * math.sin(phase),
        z,
    )


def generate_circle_points(center, radius, period_s, command_period_s, z):
    count = max(4, int(math.ceil(period_s / command_period_s)))
    return [
        circle_target(center, radius, period_s, i * period_s / count, z)
        for i in range(count + 1)
    ]


def generate_figure8_points(center, radius_x, radius_y, period_s, command_period_s, z):
    count = max(8, int(math.ceil(period_s / command_period_s)))
    return [
        figure8_target(center, radius_x, radius_y, period_s, i * period_s / count, z)
        for i in range(count + 1)
    ]


def setup_estimate_logger(cf, LogConfig, estimate_state):
    logconf = LogConfig(name='AutonomyLadder', period_in_ms=LOG_PERIOD_MS)
    logconf.add_variable('pm.vbat', 'float')
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')

    def on_data(timestamp, data, logconf):
        del timestamp, logconf
        estimate_state.update(
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z'],
            data['pm.vbat'],
        )

    def on_error(logconf, msg):
        print(f"[WARN] Logger error from {logconf.name}: {msg}")

    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(on_data)
    logconf.error_cb.add_callback(on_error)
    logconf.start()
    return logconf


def send_extpose_or_extpos(cf, pose_mode, x, y, z, quat):
    if pose_mode == 'extpose':
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)


def wait_for_fresh_pose(mocap_state, timeout, pose_stale_timeout):
    print("[INFO] Waiting for fresh mocap pose...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if pose_age(mocap_state) <= pose_stale_timeout:
            position, quat, _, frames, _ = mocap_state.snapshot()
            print(
                "[MOCAP] Fresh pose: "
                f"pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
                f"quat=({quat.x:.3f}, {quat.y:.3f}, {quat.z:.3f}, {quat.w:.3f}) "
                f"frames={frames}"
            )
            return position
        time.sleep(0.05)
    raise RuntimeError("No fresh mocap pose received before timeout")


def require_stable_pose(mocap_state, args):
    print(f"[INFO] Checking pose stability for {POSE_STABILITY_SECONDS:.1f}s...")
    samples = []
    started_at = time.time()
    while time.time() - started_at < POSE_STABILITY_SECONDS:
        if pose_age(mocap_state) > args.pose_stale_timeout:
            raise RuntimeError("Mocap pose became stale during stability check")
        position, _, _, _, _ = mocap_state.snapshot()
        if position is not None:
            samples.append(position)
        time.sleep(0.05)
    if len(samples) < 5:
        raise RuntimeError("Not enough mocap samples for stability check")
    ranges = []
    for axis in range(3):
        axis_values = [sample[axis] for sample in samples]
        ranges.append(max(axis_values) - min(axis_values))
    print(
        "[INFO] Pose range: "
        f"dx={ranges[0]:.3f}, dy={ranges[1]:.3f}, dz={ranges[2]:.3f}"
    )
    if any(axis_range > POSE_STABILITY_MAX_RANGE for axis_range in ranges):
        raise RuntimeError(
            "Mocap pose is not stable enough "
            f"(limit {POSE_STABILITY_MAX_RANGE:.3f}m)"
        )


def require_battery(estimate_state, min_battery):
    _, battery, _ = estimate_state.snapshot()
    print(f"[INFO] Battery: {battery:.2f} V")
    if battery <= 0.0:
        raise RuntimeError("Battery voltage was not logged")
    if battery < min_battery:
        raise RuntimeError(
            f"Battery {battery:.2f}V is below minimum {min_battery:.2f}V"
        )
    if battery < LOW_BATTERY_VOLTAGE:
        print("[WARN] Battery is low; expect reduced thrust authority.")


def require_start_inside_bounds(start_position, bounds):
    safe, reason = bounds.check(start_position)
    print(f"[INFO] Start position bounds: {reason}")
    if not safe:
        raise RuntimeError(f"Start position is outside configured bounds: {reason}")


def estimate_position_error(mocap_state, estimate_state):
    mocap_position, _, _, _, _ = mocap_state.snapshot()
    estimate_position, _, estimate_time = estimate_state.snapshot()
    if mocap_position is None or estimate_position is None:
        return None
    return distance_3d(mocap_position, estimate_position), time.time() - estimate_time


def require_estimator_agreement(mocap_state, estimate_state, logger, stats, args):
    print(
        "[INFO] Verifying estimator agrees with mocap "
        f"for {ESTIMATE_MATCH_SECONDS:.1f}s..."
    )
    deadline = time.time() + ESTIMATE_MATCH_SECONDS
    last_report = 0.0
    while time.time() < deadline:
        if pose_age(mocap_state) > args.pose_stale_timeout:
            raise RuntimeError("Mocap pose went stale during estimator agreement check")
        error_and_age = estimate_position_error(mocap_state, estimate_state)
        if error_and_age is None:
            time.sleep(0.05)
            continue
        error, age = error_and_age
        log_sample(
            logger,
            stats,
            mocap_state,
            estimate_state,
            args.start_position,
            phase='preflight-estimator',
            target=args.start_position,
            command='estimator-agreement',
            stop_reason='',
            guard_ok=error <= args.estimate_tolerance,
        )
        if age > ESTIMATE_MAX_AGE:
            raise RuntimeError(
                f"stateEstimate age {age:.2f}s exceeded {ESTIMATE_MAX_AGE:.2f}s"
            )
        if time.time() - last_report >= 0.25:
            print(f"[INFO] Estimator error={error:.3f}m")
            last_report = time.time()
        if error > args.estimate_tolerance:
            raise RuntimeError(
                f"Estimator disagreement {error:.3f}m exceeded "
                f"{args.estimate_tolerance:.3f}m"
            )
        time.sleep(0.05)


def log_sample(
    logger,
    stats,
    mocap_state,
    estimate_state,
    start_position,
    phase,
    target,
    command,
    stop_reason,
    guard_ok,
):
    now = time.time()
    mocap_position, quat, mocap_time, frame_count, _ = mocap_state.snapshot()
    estimate_position, battery, estimate_time = estimate_state.snapshot()
    mocap_age_s = now - mocap_time if mocap_time else float('inf')
    estimate_age_s = now - estimate_time if estimate_time else float('inf')

    if mocap_position is None:
        mocap_position = (float('nan'), float('nan'), float('nan'))
    if estimate_position is None:
        estimate_position = (float('nan'), float('nan'), float('nan'))
    if quat is None:
        quat_values = (float('nan'), float('nan'), float('nan'), float('nan'))
    else:
        quat_values = (quat.x, quat.y, quat.z, quat.w)

    estimate_error = (
        distance_3d(mocap_position, estimate_position)
        if all(math.isfinite(v) for v in mocap_position + estimate_position)
        else float('nan')
    )
    height_above_start = (
        mocap_position[2] - start_position[2]
        if all(math.isfinite(v) for v in mocap_position)
        else float('nan')
    )
    radius_from_start = (
        distance_2d(mocap_position, start_position)
        if all(math.isfinite(v) for v in mocap_position)
        else float('nan')
    )
    target_error = (
        distance_3d(mocap_position, target)
        if target is not None and all(math.isfinite(v) for v in mocap_position)
        else float('nan')
    )
    stats.update(estimate_error, height_above_start, radius_from_start, target_error, battery)
    logger.write({
        'phase': phase,
        'command': command,
        'stop_reason': stop_reason,
        'target_x': target[0] if target is not None else '',
        'target_y': target[1] if target is not None else '',
        'target_z': target[2] if target is not None else '',
        'target_yaw_deg': 0.0,
        'mocap_x': mocap_position[0],
        'mocap_y': mocap_position[1],
        'mocap_z': mocap_position[2],
        'mocap_qx': quat_values[0],
        'mocap_qy': quat_values[1],
        'mocap_qz': quat_values[2],
        'mocap_qw': quat_values[3],
        'mocap_age_s': mocap_age_s,
        'mocap_frame_count': frame_count,
        'estimate_x': estimate_position[0],
        'estimate_y': estimate_position[1],
        'estimate_z': estimate_position[2],
        'estimate_age_s': estimate_age_s,
        'estimate_error_m': estimate_error,
        'battery_v': battery,
        'height_above_start_m': height_above_start,
        'radius_from_start_m': radius_from_start,
        'target_error_m': target_error,
        'guard_ok': int(guard_ok),
    })


def log_stop_sample(
    logger,
    stats,
    mocap_state,
    estimate_state,
    start_position,
    stop_reason,
):
    if start_position is None:
        return
    log_sample(
        logger,
        stats,
        mocap_state,
        estimate_state,
        start_position,
        phase='stop',
        target=start_position,
        command='stop',
        stop_reason=stop_reason,
        guard_ok=False,
    )


def check_runtime_guards(mocap_state, estimate_state, start_position, bounds, args, apply_flight_limits):
    if pose_age(mocap_state) > args.pose_stale_timeout:
        raise GuardTrip("mocap pose stale", immediate_stop=True)

    error_and_age = estimate_position_error(mocap_state, estimate_state)
    if error_and_age is None:
        raise GuardTrip("missing estimator position", immediate_stop=True)
    error, age = error_and_age
    if age > ESTIMATE_MAX_AGE:
        raise GuardTrip(f"stateEstimate stale: {age:.2f}s", immediate_stop=True)
    if error > args.estimate_tolerance:
        raise GuardTrip(
            f"estimator disagreement {error:.3f}m exceeded {args.estimate_tolerance:.3f}m",
            immediate_stop=True,
        )

    position, _, _, _, _ = mocap_state.snapshot()
    safe, reason = bounds.check(position)
    if not safe:
        raise GuardTrip(f"boundary guard tripped: {reason}", immediate_stop=False)

    if not apply_flight_limits:
        return

    height_above_start = position[2] - start_position[2]
    radius_from_start = distance_2d(position, start_position)
    if height_above_start > args.max_height_above_start:
        raise GuardTrip(
            "height above start "
            f"{height_above_start:.3f}m exceeded {args.max_height_above_start:.3f}m",
            immediate_stop=False,
        )
    if radius_from_start > args.max_radius_from_start:
        raise GuardTrip(
            "radius from start "
            f"{radius_from_start:.3f}m exceeded {args.max_radius_from_start:.3f}m",
            immediate_stop=False,
        )


def monitor(
    args,
    logger,
    stats,
    key_reader,
    mocap_state,
    estimate_state,
    start_position,
    bounds,
    phase,
    target,
    command,
    duration,
    apply_flight_limits,
):
    deadline = time.time() + duration
    while time.time() < deadline:
        key = key_reader.poll_stop()
        if key is not None:
            log_sample(
                logger,
                stats,
                mocap_state,
                estimate_state,
                start_position,
                phase,
                target,
                command,
                stop_reason='operator abort',
                guard_ok=False,
            )
            raise OperatorAbort("operator requested immediate stop")

        check_runtime_guards(
            mocap_state,
            estimate_state,
            start_position,
            bounds,
            args,
            apply_flight_limits=apply_flight_limits,
        )
        log_sample(
            logger,
            stats,
            mocap_state,
            estimate_state,
            start_position,
            phase,
            target,
            command,
            stop_reason='',
            guard_ok=True,
        )
        time.sleep(args.command_period)


def command_go_to_and_monitor(
    cf,
    args,
    logger,
    stats,
    key_reader,
    mocap_state,
    estimate_state,
    start_position,
    bounds,
    phase,
    target,
    duration,
):
    cf.high_level_commander.go_to(target[0], target[1], target[2], 0.0, duration, relative=False)
    monitor(
        args,
        logger,
        stats,
        key_reader,
        mocap_state,
        estimate_state,
        start_position,
        bounds,
        phase=phase,
        target=target,
        command='go_to',
        duration=duration,
        apply_flight_limits=True,
    )


def validate_path_inside_bounds(points, bounds, start_position, args):
    safe, reason = bounds.contains_path(points)
    if not safe:
        raise RuntimeError(f"Path leaves configured bounds before flight: {reason}")
    for point in points:
        if point[2] - start_position[2] > args.max_height_above_start:
            raise RuntimeError("Path exceeds max height above start before flight")
        if distance_2d(point, start_position) > args.max_radius_from_start:
            raise RuntimeError("Path exceeds max radius from start before flight")


def hover_target(start_position, args):
    return (start_position[0], start_position[1], start_position[2] + args.height)


def step_targets(start_position, args):
    z = start_position[2] + args.height
    center = (start_position[0], start_position[1], z)
    return [
        (start_position[0] + args.step_distance, start_position[1], z),
        center,
        (start_position[0] - args.step_distance, start_position[1], z),
        center,
        (start_position[0], start_position[1] + args.step_distance, z),
        center,
        (start_position[0], start_position[1] - args.step_distance, z),
        center,
    ]


def circle_points(start_position, args):
    return generate_circle_points(
        (start_position[0], start_position[1]),
        args.circle_radius,
        args.path_period,
        args.path_command_period,
        start_position[2] + args.height,
    )


def figure8_points(start_position, args):
    return generate_figure8_points(
        (start_position[0], start_position[1]),
        args.figure8_radius_x,
        args.figure8_radius_y,
        args.path_period,
        args.path_command_period,
        start_position[2] + args.height,
    )


def planned_flight_points(mode, start_position, args):
    points = [hover_target(start_position, args)]
    if mode == 'hover':
        return points
    if mode == 'steps':
        return points + step_targets(start_position, args)
    if mode == 'circle':
        return points + circle_points(start_position, args)
    if mode == 'figure8':
        return points + figure8_points(start_position, args)
    return []


def validate_planned_flight(mode, start_position, bounds, args):
    points = planned_flight_points(mode, start_position, args)
    if not points:
        return
    validate_path_inside_bounds(points, bounds, start_position, args)
    print(f"[INFO] Planned {mode} path validated: {len(points)} target points")


def normal_land(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds):
    target = (start_position[0], start_position[1], start_position[2])
    print("[SAFETY] Landing to start z...")
    cf.high_level_commander.land(start_position[2], args.land_duration)
    monitor(
        args,
        logger,
        stats,
        key_reader,
        mocap_state,
        estimate_state,
        start_position,
        bounds,
        phase='land',
        target=target,
        command='land',
        duration=args.land_duration + 0.5,
        apply_flight_limits=True,
    )
    cf.high_level_commander.stop()


def immediate_stop_and_disarm(cf):
    print("[SAFETY] Immediate stop/disarm")
    try:
        cf.high_level_commander.stop()
    except Exception:
        pass
    try:
        cf.commander.send_stop_setpoint()
    except Exception:
        pass
    cf.platform.send_arming_request(False)


def guarded_land_or_stop(cf, args, start_position):
    print("[SAFETY] Guarded landing attempt")
    try:
        cf.high_level_commander.land(start_position[2], args.land_duration)
        time.sleep(args.land_duration + 0.5)
        cf.high_level_commander.stop()
    except Exception as exc:
        print(f"[WARN] Guarded landing failed: {exc}")
        try:
            cf.high_level_commander.stop()
        except Exception:
            pass
        try:
            cf.commander.send_stop_setpoint()
        except Exception:
            pass
    finally:
        cf.platform.send_arming_request(False)


def fly_hover(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds):
    target = hover_target(start_position, args)
    print(f"[FLIGHT] Takeoff to z={target[2]:.3f}")
    cf.high_level_commander.takeoff(target[2], args.takeoff_duration)
    monitor(
        args,
        logger,
        stats,
        key_reader,
        mocap_state,
        estimate_state,
        start_position,
        bounds,
        phase='takeoff',
        target=target,
        command='takeoff',
        duration=args.takeoff_duration + 0.5,
        apply_flight_limits=True,
    )
    print(f"[FLIGHT] Hover for {args.hover_duration:.1f}s")
    monitor(
        args,
        logger,
        stats,
        key_reader,
        mocap_state,
        estimate_state,
        start_position,
        bounds,
        phase='hover',
        target=target,
        command='hold',
        duration=args.hover_duration,
        apply_flight_limits=True,
    )


def fly_steps(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds):
    fly_hover(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds)
    targets = step_targets(start_position, args)
    for index, target in enumerate(targets, start=1):
        print(f"[FLIGHT] Step {index}/{len(targets)} to ({target[0]:.3f}, {target[1]:.3f})")
        command_go_to_and_monitor(
            cf,
            args,
            logger,
            stats,
            key_reader,
            mocap_state,
            estimate_state,
            start_position,
            bounds,
            phase='steps',
            target=target,
            duration=args.step_duration,
        )


def fly_path_points(
    cf,
    args,
    logger,
    stats,
    key_reader,
    mocap_state,
    estimate_state,
    start_position,
    bounds,
    phase,
    points,
):
    for index, target in enumerate(points, start=1):
        print(f"[FLIGHT] {phase} point {index}/{len(points)}")
        command_go_to_and_monitor(
            cf,
            args,
            logger,
            stats,
            key_reader,
            mocap_state,
            estimate_state,
            start_position,
            bounds,
            phase=phase,
            target=target,
            duration=args.path_command_period,
        )


def fly_circle(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds):
    fly_hover(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds)
    points = circle_points(start_position, args)
    fly_path_points(
        cf,
        args,
        logger,
        stats,
        key_reader,
        mocap_state,
        estimate_state,
        start_position,
        bounds,
        phase='circle',
        points=points,
    )


def fly_figure8(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds):
    fly_hover(cf, args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds)
    points = figure8_points(start_position, args)
    fly_path_points(
        cf,
        args,
        logger,
        stats,
        key_reader,
        mocap_state,
        estimate_state,
        start_position,
        bounds,
        phase='figure8',
        points=points,
    )


def run_validate(args, logger, stats, key_reader, mocap_state, estimate_state, start_position, bounds):
    print(f"[VALIDATE] Logging estimator agreement for {args.validate_duration:.1f}s")
    monitor(
        args,
        logger,
        stats,
        key_reader,
        mocap_state,
        estimate_state,
        start_position,
        bounds,
        phase='validate',
        target=start_position,
        command='log-only',
        duration=args.validate_duration,
        apply_flight_limits=False,
    )


def print_summary(success, stop_reason, output_path, stats):
    print("=" * 72)
    print("[SUMMARY]")
    print(f"success: {success}")
    print(f"stop_reason: {stop_reason or 'none'}")
    print(f"log: {output_path}")
    print(f"rows: {stats.rows}")
    print(f"max_estimate_error_m: {stats.max_estimate_error:.3f}")
    print(f"max_height_above_start_m: {stats.max_height_above_start:.3f}")
    print(f"max_radius_from_start_m: {stats.max_radius_from_start:.3f}")
    print(f"max_target_error_m: {stats.max_target_error:.3f}")
    min_battery = stats.min_battery if stats.min_battery is not None else float('nan')
    print(f"min_battery_v: {min_battery:.2f}")
    print("=" * 72)


def validate_args(args):
    if args.height <= 0.0:
        raise ValueError("--height must be greater than zero")
    if args.max_height_above_start <= 0.0:
        raise ValueError("--max-height-above-start must be greater than zero")
    if args.max_radius_from_start <= 0.0:
        raise ValueError("--max-radius-from-start must be greater than zero")
    if args.estimate_tolerance <= 0.0:
        raise ValueError("--estimate-tolerance must be greater than zero")
    if args.pose_stale_timeout <= 0.0:
        raise ValueError("--pose-stale-timeout must be greater than zero")
    if args.command_period <= 0.0:
        raise ValueError("--command-period must be greater than zero")
    if args.path_command_period <= 0.0:
        raise ValueError("--path-command-period must be greater than zero")
    if args.path_period <= 0.0:
        raise ValueError("--path-period must be greater than zero")
    if args.x_min >= args.x_max or args.y_min >= args.y_max or args.z_min >= args.z_max:
        raise ValueError("Bounds min values must be less than max values")


def run(args):
    validate_args(args)
    runtime = load_runtime_modules()
    runtime['cflib_crtp'].init_drivers()

    bounds = make_bounds(args)
    output_path = make_output_path(args.output, args.mode)
    mocap_state = MocapState()
    estimate_state = EstimateState()
    logger = CsvLogger(output_path, args.mode)
    stats = RunStats()
    mocap_reader = MocapReader(
        runtime['motioncapture'],
        args.host,
        args.body,
        args.pose_mode,
        mocap_state,
    )
    estimate_logconf = None
    cf = None
    armed = False
    success = False
    stop_reason = ''

    print("=" * 72)
    print("AIMSLAB MOCAP AUTONOMY LADDER")
    print("=" * 72)
    print(f"Mode: {args.mode}")
    print(f"URI: {args.uri}")
    print(f"Rigid body: {args.body}@{args.host}")
    print(f"Pose stream: {args.pose_mode}")
    print(f"Output: {output_path}")
    print("Use validate -> hover -> steps -> circle -> figure8.")
    print("q, Esc, or Space request immediate stop/disarm during flight modes.")
    print("=" * 72)

    try:
        input("Press ENTER to connect mocap and Crazyflie, or Ctrl+C to abort...")
        mocap_reader.start()
        start_position = wait_for_fresh_pose(mocap_state, MOCAP_TIMEOUT, args.pose_stale_timeout)
        args.start_position = start_position
        require_stable_pose(mocap_state, args)
        require_start_inside_bounds(start_position, bounds)
        validate_planned_flight(args.mode, start_position, bounds, args)

        with runtime['SyncCrazyflie'](args.uri, cf=runtime['Crazyflie'](rw_cache='./cache')) as scf:
            cf = scf.cf
            print("[INFO] Crazyflie connected.")
            estimate_logconf = setup_estimate_logger(cf, runtime['LogConfig'], estimate_state)
            time.sleep(0.8)
            require_battery(estimate_state, args.min_battery)

            mocap_reader.on_pose = lambda x, y, z, quat: send_extpose_or_extpos(
                cf,
                args.pose_mode,
                x,
                y,
                z,
                quat,
            )
            print("[INFO] Configuring estimator and high-level commander...")
            if args.pose_mode == 'extpose':
                cf.param.set_value('locSrv.extQuatStdDev', args.orientation_std_dev)
            cf.param.set_value('stabilizer.estimator', '2')
            cf.param.set_value('commander.enHighLevel', '1')
            time.sleep(0.5)

            print("[INFO] Resetting estimator while external pose is streaming...")
            runtime['reset_estimator'](cf)
            time.sleep(1.0)
            require_estimator_agreement(mocap_state, estimate_state, logger, stats, args)

            if args.mode == 'validate':
                with TerminalKeyReader() as key_reader:
                    run_validate(
                        args,
                        logger,
                        stats,
                        key_reader,
                        mocap_state,
                        estimate_state,
                        start_position,
                        bounds,
                    )
                success = True
                return

            input("Press ENTER to arm and run flight mode, or Ctrl+C to abort...")
            print("[INFO] Arming...")
            cf.platform.send_arming_request(True)
            armed = True
            time.sleep(1.0)

            with TerminalKeyReader() as key_reader:
                if args.mode == 'hover':
                    fly_hover(
                        cf,
                        args,
                        logger,
                        stats,
                        key_reader,
                        mocap_state,
                        estimate_state,
                        start_position,
                        bounds,
                    )
                elif args.mode == 'steps':
                    fly_steps(
                        cf,
                        args,
                        logger,
                        stats,
                        key_reader,
                        mocap_state,
                        estimate_state,
                        start_position,
                        bounds,
                    )
                elif args.mode == 'circle':
                    fly_circle(
                        cf,
                        args,
                        logger,
                        stats,
                        key_reader,
                        mocap_state,
                        estimate_state,
                        start_position,
                        bounds,
                    )
                elif args.mode == 'figure8':
                    fly_figure8(
                        cf,
                        args,
                        logger,
                        stats,
                        key_reader,
                        mocap_state,
                        estimate_state,
                        start_position,
                        bounds,
                    )
                normal_land(
                    cf,
                    args,
                    logger,
                    stats,
                    key_reader,
                    mocap_state,
                    estimate_state,
                    start_position,
                    bounds,
                )
            cf.platform.send_arming_request(False)
            armed = False
            success = True
    except OperatorAbort as exc:
        stop_reason = str(exc)
        log_stop_sample(
            logger,
            stats,
            mocap_state,
            estimate_state,
            getattr(args, 'start_position', None),
            stop_reason,
        )
        if cf is not None:
            immediate_stop_and_disarm(cf)
        armed = False
    except GuardTrip as exc:
        stop_reason = exc.reason
        log_stop_sample(
            logger,
            stats,
            mocap_state,
            estimate_state,
            getattr(args, 'start_position', None),
            stop_reason,
        )
        if cf is not None and armed:
            if exc.immediate_stop:
                immediate_stop_and_disarm(cf)
            else:
                guarded_land_or_stop(cf, args, args.start_position)
            armed = False
    except KeyboardInterrupt:
        stop_reason = 'keyboard interrupt'
        print("\n[INTERRUPT] Aborted by user")
        log_stop_sample(
            logger,
            stats,
            mocap_state,
            estimate_state,
            getattr(args, 'start_position', None),
            stop_reason,
        )
        if cf is not None and armed:
            immediate_stop_and_disarm(cf)
            armed = False
    except Exception as exc:
        stop_reason = str(exc)
        print(f"\n[ERROR] {exc}")
        if cf is not None and armed:
            immediate_stop_and_disarm(cf)
            armed = False
        raise
    finally:
        if cf is not None and armed:
            immediate_stop_and_disarm(cf)
        if estimate_logconf is not None:
            try:
                estimate_logconf.stop()
            except Exception as exc:
                print(f"[WARN] Estimate logger stop failed: {exc}")
        mocap_reader.close()
        logger.close()
        print_summary(success, stop_reason, output_path, stats)


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'mode',
        choices=('validate', 'hover', 'steps', 'circle', 'figure8'),
        help='Autonomy ladder stage to run.',
    )
    parser.add_argument('--uri', default=DEFAULT_URI)
    parser.add_argument('--host', default=DEFAULT_HOST_NAME)
    parser.add_argument('--body', default=DEFAULT_RIGID_BODY_NAME)
    parser.add_argument('--output', default=None)
    parser.add_argument('--pose-mode', choices=('extpose', 'extpos'), default='extpose')
    parser.add_argument('--orientation-std-dev', type=float, default=8.0e-3)
    parser.add_argument('--height', type=float, default=0.35)
    parser.add_argument('--pose-stale-timeout', type=float, default=0.30)
    parser.add_argument('--estimate-tolerance', type=float, default=0.08)
    parser.add_argument('--max-radius-from-start', type=float, default=0.45)
    parser.add_argument('--max-height-above-start', type=float, default=0.60)
    parser.add_argument('--min-battery', type=float, default=3.75)
    parser.add_argument('--validate-duration', type=float, default=30.0)
    parser.add_argument('--takeoff-duration', type=float, default=4.0)
    parser.add_argument('--hover-duration', type=float, default=15.0)
    parser.add_argument('--land-duration', type=float, default=3.0)
    parser.add_argument('--step-distance', type=float, default=0.10)
    parser.add_argument('--step-duration', type=float, default=3.0)
    parser.add_argument('--circle-radius', type=float, default=0.05)
    parser.add_argument('--figure8-radius-x', type=float, default=0.06)
    parser.add_argument('--figure8-radius-y', type=float, default=0.05)
    parser.add_argument('--path-period', type=float, default=24.0)
    parser.add_argument('--path-command-period', type=float, default=0.75)
    parser.add_argument('--command-period', type=float, default=COMMAND_PERIOD)
    parser.add_argument('--x-min', type=float, default=DEFAULT_X_MIN)
    parser.add_argument('--x-max', type=float, default=DEFAULT_X_MAX)
    parser.add_argument('--y-min', type=float, default=DEFAULT_Y_MIN)
    parser.add_argument('--y-max', type=float, default=DEFAULT_Y_MAX)
    parser.add_argument('--z-min', type=float, default=DEFAULT_Z_MIN)
    parser.add_argument('--z-max', type=float, default=DEFAULT_Z_MAX)
    parser.add_argument('--boundary-margin', type=float, default=0.10)
    return parser.parse_args()


if __name__ == '__main__':
    run(parse_args())
