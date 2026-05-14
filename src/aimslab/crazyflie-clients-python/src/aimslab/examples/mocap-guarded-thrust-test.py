"""
Guarded raw-thrust mocap test for Crazyflie.

This keeps the same mocap streaming and safety checks as the guarded takeoff
test, but uses low-level thrust setpoints instead of the high-level commander.
It is intended for very small vertical tests only.
"""
import math
import time
from threading import Lock
from threading import Thread

import cflib.crtp
import motioncapture
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator


# Match the known-working controller script unless CFLIB_URI is explicitly set.
uri = uri_helper.uri_from_env(default='radio://0/80/2M')

# Mocap settings for the AIMSLab Motive/VRPN setup.
host_name = '192.168.1.42:3883'
mocap_system_type = 'vrpn'
rigid_body_name = 'crazyflie_21'
send_full_pose = True
orientation_std_dev = 8.0e-3

# Keep raw-thrust testing close to the known-working controller script. Mocap is
# still used by this process for live safety guards, but external pose is not fed
# into the Crazyflie unless this is explicitly enabled.
FEED_MOCAP_TO_CRAZYFLIE = False
USE_KALMAN_ESTIMATOR = False
WRITE_COMMANDER_PARAMS = False

# Conservative cage limits in mocap/world coordinates. Update these after
# boundary mapping is trustworthy.
CAGE_BOUNDS = {
    'x_min': -1.5,
    'x_max': 1.5,
    'y_min': -1.5,
    'y_max': 1.5,
    'z_min': 0.0,
    'z_max': 2.0,
}
SAFETY_MARGIN = 0.20

# The rigid body currently reports the floor near z=0.037 at cage center.
FLOOR_Z = 0.037
TARGET_HEIGHT_ABOVE_FLOOR = 0.12
TARGET_Z = FLOOR_Z + TARGET_HEIGHT_ABOVE_FLOOR

# Raw Crazyflie thrust is in the 0..65535 range. The cap and trim below are
# tuned to the user's manually validated low-hover test window.
MIN_THRUST = 0
START_THRUST = 75000
MAX_THRUST = 79000
THRUST_STEP = 100
RAMP_INTERVAL = 0.02
HOVER_DURATION = 1.0
CUT_THRUST_STEPS = 8
COMMAND_PRIME_SECONDS = 1.0
CONTROL_MODE = 'raw_ramp'
MANUAL_THRUST_PERCENT = 57.0

ROLL_DEG = 0.0
PITCH_DEG = 0.0
YAWRATE_DEG_PER_S = 0.0

POSE_STALE_TIMEOUT = 0.30
POSE_STABILITY_SECONDS = 2.0
POSE_STABILITY_MAX_RANGE = 0.05
MAX_HORIZONTAL_DRIFT = 0.30
ESTIMATE_MATCH_SECONDS = 1.0
ESTIMATE_MATCH_TOLERANCE = 0.05
ESTIMATE_MAX_AGE = 0.30
PREFLIGHT_COUNTDOWN_SECONDS = 3


class MocapState:
    def __init__(self):
        self._lock = Lock()
        self.position = None
        self.quat = None
        self.last_update = 0.0
        self.frame_count = 0

    def update(self, position, quat):
        with self._lock:
            self.position = tuple(position)
            self.quat = quat
            self.last_update = time.time()
            self.frame_count += 1

    def snapshot(self):
        with self._lock:
            return self.position, self.quat, self.last_update, self.frame_count


class EstimateState:
    def __init__(self):
        self._lock = Lock()
        self.position = None
        self.last_update = 0.0

    def update(self, x, y, z):
        with self._lock:
            self.position = (x, y, z)
            self.last_update = time.time()

    def snapshot(self):
        with self._lock:
            return self.position, self.last_update


mocap_state = MocapState()
estimate_state = EstimateState()


class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)
        self.daemon = True
        self.body_name = body_name
        self.on_pose = None
        self.error = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        try:
            mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
            print(f"[INFO] Mocap connected, looking for '{self.body_name}'")
            found_body = False

            while self._stay_open:
                mc.waitForNextFrame()
                for name, obj in mc.rigidBodies.items():
                    if name != self.body_name:
                        continue

                    if not found_body:
                        print(f"[INFO] Found and tracking rigid body: {name}")
                        found_body = True

                    pos = obj.position
                    quat = obj.rotation
                    mocap_state.update((pos[0], pos[1], pos[2]), quat)

                    if self.on_pose:
                        self.on_pose(pos[0], pos[1], pos[2], quat)
        except Exception as exc:
            self.error = exc


def send_extpose_quat(cf, x, y, z, quat):
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')


def disable_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '0')


def position_safe(position):
    x, y, z = position
    if x < CAGE_BOUNDS['x_min'] + SAFETY_MARGIN:
        return False, f"x={x:.3f} is too close to x_min"
    if x > CAGE_BOUNDS['x_max'] - SAFETY_MARGIN:
        return False, f"x={x:.3f} is too close to x_max"
    if y < CAGE_BOUNDS['y_min'] + SAFETY_MARGIN:
        return False, f"y={y:.3f} is too close to y_min"
    if y > CAGE_BOUNDS['y_max'] - SAFETY_MARGIN:
        return False, f"y={y:.3f} is too close to y_max"
    if z < CAGE_BOUNDS['z_min']:
        return False, f"z={z:.3f} is below z_min"
    if z > CAGE_BOUNDS['z_max'] - SAFETY_MARGIN:
        return False, f"z={z:.3f} is too close to z_max"
    return True, "inside bounds"


def get_pose_age():
    _, _, last_update, _ = mocap_state.snapshot()
    if last_update == 0.0:
        return float('inf')
    return time.time() - last_update


def wait_for_fresh_pose(timeout=8.0):
    print("[INFO] Waiting for fresh mocap pose...")
    deadline = time.time() + timeout
    while time.time() < deadline:
        if get_pose_age() <= POSE_STALE_TIMEOUT:
            position, quat, _, frames = mocap_state.snapshot()
            print_pose("[MOCAP] Fresh pose", position, quat, frames)
            return position
        time.sleep(0.05)

    raise RuntimeError("No fresh mocap pose received before timeout")


def print_pose(prefix, position, quat, frames=None):
    frame_text = f" frames={frames}" if frames is not None else ""
    print(
        f"{prefix}: pos=({position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}) "
        f"quat=({quat.x:.3f}, {quat.y:.3f}, {quat.z:.3f}, {quat.w:.3f}){frame_text}"
    )


def require_stable_pose():
    print(f"[INFO] Checking pose stability for {POSE_STABILITY_SECONDS:.1f}s...")
    samples = []
    start_time = time.time()

    while time.time() - start_time < POSE_STABILITY_SECONDS:
        if get_pose_age() > POSE_STALE_TIMEOUT:
            raise RuntimeError("Mocap pose became stale during stability check")

        position, _, _, _ = mocap_state.snapshot()
        if position is not None:
            samples.append(position)
        time.sleep(0.05)

    if len(samples) < 5:
        raise RuntimeError("Not enough mocap samples for stability check")

    ranges = []
    for axis in range(3):
        values = [sample[axis] for sample in samples]
        ranges.append(max(values) - min(values))

    print(f"[INFO] Pose range: dx={ranges[0]:.3f}, dy={ranges[1]:.3f}, dz={ranges[2]:.3f}")
    if any(axis_range > POSE_STABILITY_MAX_RANGE for axis_range in ranges):
        raise RuntimeError(
            "Mocap pose is not stable enough for thrust test "
            f"(limit {POSE_STABILITY_MAX_RANGE:.3f}m)"
        )


def setup_estimate_logger(cf):
    logconf = LogConfig(name='StateEstimate', period_in_ms=200)
    logconf.add_variable('stateEstimate.x', 'float')
    logconf.add_variable('stateEstimate.y', 'float')
    logconf.add_variable('stateEstimate.z', 'float')

    def on_data(timestamp, data, logconf):
        del timestamp
        del logconf
        estimate_state.update(
            data['stateEstimate.x'],
            data['stateEstimate.y'],
            data['stateEstimate.z'],
        )

    def on_error(logconf, msg):
        print(f"[WARN] Estimate logger error from {logconf.name}: {msg}")

    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(on_data)
    logconf.error_cb.add_callback(on_error)
    logconf.start()
    return logconf


def print_estimate_comparison():
    mocap_position, _, _, _ = mocap_state.snapshot()
    estimate_position, estimate_time = estimate_state.snapshot()
    if estimate_position is None:
        print("[WARN] No stateEstimate position has been logged yet")
        return

    age = time.time() - estimate_time
    print(
        "[ESTIMATE] "
        f"mocap=({mocap_position[0]:.3f}, {mocap_position[1]:.3f}, {mocap_position[2]:.3f}) "
        f"estimate=({estimate_position[0]:.3f}, {estimate_position[1]:.3f}, {estimate_position[2]:.3f}) "
        f"age={age:.2f}s"
    )


def estimate_position_error():
    mocap_position, _, _, _ = mocap_state.snapshot()
    estimate_position, estimate_time = estimate_state.snapshot()
    if mocap_position is None or estimate_position is None:
        return None, None, None

    error = math.sqrt(
        (mocap_position[0] - estimate_position[0]) ** 2
        + (mocap_position[1] - estimate_position[1]) ** 2
        + (mocap_position[2] - estimate_position[2]) ** 2
    )
    age = time.time() - estimate_time
    return error, mocap_position, age


def require_estimate_agreement():
    print(
        "[INFO] Verifying estimator agrees with mocap "
        f"for {ESTIMATE_MATCH_SECONDS:.1f}s..."
    )
    deadline = time.time() + ESTIMATE_MATCH_SECONDS
    last_report = 0.0

    while time.time() < deadline:
        if get_pose_age() > POSE_STALE_TIMEOUT:
            raise RuntimeError("Mocap pose went stale during estimator agreement check")

        error, _, age = estimate_position_error()
        if error is None:
            raise RuntimeError("No stateEstimate position available for agreement check")

        if age > ESTIMATE_MAX_AGE:
            raise RuntimeError(
                f"stateEstimate age {age:.2f}s exceeded {ESTIMATE_MAX_AGE:.2f}s"
            )

        if time.time() - last_report >= 0.25:
            print_estimate_comparison()
            print(
                f"[INFO] Estimator error={error:.3f}m "
                f"(limit {ESTIMATE_MATCH_TOLERANCE:.3f}m)"
            )
            last_report = time.time()

        if error > ESTIMATE_MATCH_TOLERANCE:
            raise RuntimeError(
                f"Estimator disagreement {error:.3f}m exceeded "
                f"{ESTIMATE_MATCH_TOLERANCE:.3f}m"
            )

        time.sleep(0.05)


def horizontal_distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def require_live_guards(start_position):
    if get_pose_age() > POSE_STALE_TIMEOUT:
        raise RuntimeError("Mocap pose went stale during thrust test")

    position, quat, _, _ = mocap_state.snapshot()
    if position is None:
        raise RuntimeError("No mocap position available during thrust test")

    is_safe, reason = position_safe(position)
    if not is_safe:
        raise RuntimeError(f"Boundary guard tripped during thrust test: {reason}")

    drift = horizontal_distance(position, start_position)
    if drift > MAX_HORIZONTAL_DRIFT:
        raise RuntimeError(
            f"Horizontal drift {drift:.3f}m exceeded {MAX_HORIZONTAL_DRIFT:.3f}m"
        )

    return position, quat, drift


def send_low_level_setpoint(cf, thrust):
    if CONTROL_MODE == 'manual_percent':
        cf.commander.send_setpoint_manual(
            ROLL_DEG,
            PITCH_DEG,
            YAWRATE_DEG_PER_S,
            MANUAL_THRUST_PERCENT,
            False,
        )
    else:
        cf.commander.send_setpoint(
            ROLL_DEG,
            PITCH_DEG,
            YAWRATE_DEG_PER_S,
            int(max(MIN_THRUST, min(MAX_THRUST, thrust))),
        )


def cut_thrust(cf, previous_thrust):
    print("[SAFETY] Cutting thrust...")
    thrust = previous_thrust
    for _ in range(CUT_THRUST_STEPS):
        thrust = max(MIN_THRUST, thrust - max(1, previous_thrust // CUT_THRUST_STEPS))
        cf.commander.send_setpoint(ROLL_DEG, PITCH_DEG, YAWRATE_DEG_PER_S, thrust)
        time.sleep(0.05)
    cf.commander.send_stop_setpoint()


def cut_thrust_and_disarm(cf, previous_thrust):
    try:
        cut_thrust(cf, previous_thrust)
    finally:
        print("[SAFETY] Disarming...")
        cf.platform.send_arming_request(False)


def preflight_countdown():
    print("[INFO] Preflight countdown starting...")
    for remaining in range(PREFLIGHT_COUNTDOWN_SECONDS, 0, -1):
        print(f"[COUNTDOWN] {remaining}...")
        time.sleep(1.0)
    print("[COUNTDOWN] Thrust ramp starting")


def prime_low_level_commander(cf):
    print(
        "[INFO] Priming low-level commander with zero-thrust setpoints "
        f"for {COMMAND_PRIME_SECONDS:.1f}s..."
    )
    deadline = time.time() + COMMAND_PRIME_SECONDS
    while time.time() < deadline:
        send_low_level_setpoint(cf, MIN_THRUST)
        time.sleep(RAMP_INTERVAL)


def run_guarded_thrust_test(cf, start_position):
    print("[FLIGHT] Starting raw-thrust ramp...")
    current_thrust = START_THRUST
    reached_target = False
    last_status = 0.0

    while current_thrust <= MAX_THRUST:
        position, quat, drift = require_live_guards(start_position)
        send_low_level_setpoint(cf, current_thrust)

        if time.time() - last_status >= 0.5:
            print(
                "[STATUS] "
                f"thrust={current_thrust} z={position[2]:.3f} "
                f"target_z={TARGET_Z:.3f} drift={drift:.3f}"
            )
            print_pose("[MOCAP]", position, quat)
            if estimate_state.snapshot()[0] is not None:
                print_estimate_comparison()
            last_status = time.time()

        if position[2] >= TARGET_Z:
            reached_target = True
            break

        current_thrust += THRUST_STEP
        time.sleep(RAMP_INTERVAL)

    if not reached_target:
        raise RuntimeError(
            f"Did not reach target z={TARGET_Z:.3f} before MAX_THRUST={MAX_THRUST}"
        )

    hold_thrust = current_thrust
    print(f"[FLIGHT] Target reached. Holding briefly at thrust={hold_thrust}...")
    hold_until = time.time() + HOVER_DURATION
    while time.time() < hold_until:
        require_live_guards(start_position)
        send_low_level_setpoint(cf, hold_thrust)
        time.sleep(0.05)

    return hold_thrust


def main():
    print("=" * 72)
    print("GUARDED MOCAP RAW-THRUST TEST")
    print("=" * 72)
    print(f"Rigid body: {rigid_body_name}@{host_name}")
    print(f"Target: z={TARGET_Z:.3f}m ({TARGET_HEIGHT_ABOVE_FLOOR:.3f}m above floor)")
    print(f"Thrust ramp: {START_THRUST}..{MAX_THRUST}, step={THRUST_STEP}")
    print(f"Control mode: {CONTROL_MODE}, manual_thrust={MANUAL_THRUST_PERCENT:.1f}%")
    print(f"Pitch trim: {PITCH_DEG:.1f}deg, countdown={PREFLIGHT_COUNTDOWN_SECONDS}s")
    print(f"Bounds: {CAGE_BOUNDS}, safety_margin={SAFETY_MARGIN:.2f}m")
    print("This script only commands vertical raw thrust with zero roll/pitch/yawrate.")
    print(
        "Mocap mode: "
        f"feed_extpose={FEED_MOCAP_TO_CRAZYFLIE}, kalman={USE_KALMAN_ESTIMATOR}"
    )
    print(f"URI: {uri}")
    print("=" * 72)

    cflib.crtp.init_drivers()
    mocap_wrapper = None
    estimate_logconf = None
    armed = False
    cf = None
    last_thrust = MIN_THRUST

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            mocap_wrapper = MocapWrapper(rigid_body_name)
            if FEED_MOCAP_TO_CRAZYFLIE:
                mocap_wrapper.on_pose = (
                    lambda x, y, z, quat: send_extpose_quat(cf, x, y, z, quat)
                )

            start_position = wait_for_fresh_pose()
            require_stable_pose()
            is_safe, reason = position_safe(start_position)
            print(f"[INFO] Start position safety: {reason}")
            if not is_safe:
                raise RuntimeError(f"Start position is outside guarded bounds: {reason}")

            input("Press ENTER to arm and run the guarded thrust ramp, or Ctrl+C to abort...")

            print("[INFO] Configuring low-level commander...")
            if WRITE_COMMANDER_PARAMS:
                disable_high_level_commander(cf)
            else:
                print("[INFO] Skipping commander param writes for controller-path diagnostic")

            if USE_KALMAN_ESTIMATOR:
                estimate_logconf = setup_estimate_logger(cf)
                print("[INFO] Configuring estimator from external mocap pose...")
                adjust_orientation_sensitivity(cf)
                activate_kalman_estimator(cf)
                print("[INFO] Resetting estimator while external pose is streaming...")
                reset_estimator(cf)
                time.sleep(1.0)
                print_estimate_comparison()
                require_estimate_agreement()
            else:
                print("[INFO] Skipping Kalman/extpose setup for raw-thrust diagnostic")

            print("[INFO] Arming...")
            cf.platform.send_arming_request(True)
            armed = True
            time.sleep(0.5)
            preflight_countdown()
            prime_low_level_commander(cf)

            last_thrust = run_guarded_thrust_test(cf, start_position)
            print("[FLIGHT] Guarded thrust test completed")
            cut_thrust_and_disarm(cf, last_thrust)
            armed = False
            print("[SUCCESS] Guarded raw-thrust test complete")
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Aborted by user")
        if armed:
            cut_thrust_and_disarm(cf, last_thrust)
    except Exception as exc:
        print(f"\n[ERROR] {exc}")
        if armed:
            cut_thrust_and_disarm(cf, last_thrust)
        raise
    finally:
        if estimate_logconf is not None:
            try:
                estimate_logconf.stop()
            except Exception as exc:
                print(f"[WARN] Estimate logger stop failed: {exc}")
        if mocap_wrapper is not None:
            mocap_wrapper.close()


if __name__ == '__main__':
    main()
