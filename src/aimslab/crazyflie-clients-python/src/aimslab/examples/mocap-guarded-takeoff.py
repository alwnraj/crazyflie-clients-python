"""
Guarded mocap takeoff test for Crazyflie.

This is the next step after confirming that VRPN pose data is available. It
streams external pose to the Crazyflie, resets the Kalman estimator, performs a
low takeoff, hovers briefly, and lands. It does not upload trajectories or
command horizontal flight.
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


uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Mocap settings for the AIMSLab Motive/VRPN setup.
host_name = '192.168.1.42:3883'
mocap_system_type = 'vrpn'
rigid_body_name = 'crazyflie_21'
send_full_pose = True
orientation_std_dev = 8.0e-3

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
TAKEOFF_HEIGHT_ABOVE_FLOOR = 0.15
TAKEOFF_Z = FLOOR_Z + TAKEOFF_HEIGHT_ABOVE_FLOOR
TAKEOFF_DURATION = 3.0
HOVER_DURATION = 2.0
LAND_Z = 0.0
LAND_DURATION = 2.0

POSE_STALE_TIMEOUT = 0.30
POSE_STABILITY_SECONDS = 2.0
POSE_STABILITY_MAX_RANGE = 0.05
MAX_HORIZONTAL_DRIFT = 0.35


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


def enable_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


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


def get_pose_age():
    _, _, last_update, _ = mocap_state.snapshot()
    if last_update == 0.0:
        return float('inf')
    return time.time() - last_update


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
            "Mocap pose is not stable enough for takeoff "
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


def horizontal_distance(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def monitor_guarded_hover(commander, start_position):
    print("[FLIGHT] Low takeoff...")
    commander.takeoff(TAKEOFF_Z, TAKEOFF_DURATION)

    monitor_until = time.time() + TAKEOFF_DURATION + HOVER_DURATION
    while time.time() < monitor_until:
        if get_pose_age() > POSE_STALE_TIMEOUT:
            raise RuntimeError("Mocap pose went stale during takeoff/hover")

        position, quat, _, _ = mocap_state.snapshot()
        is_safe, reason = position_safe(position)
        if not is_safe:
            raise RuntimeError(f"Boundary guard tripped during hover: {reason}")

        drift = horizontal_distance(position, start_position)
        if drift > MAX_HORIZONTAL_DRIFT:
            raise RuntimeError(
                f"Horizontal drift {drift:.3f}m exceeded {MAX_HORIZONTAL_DRIFT:.3f}m"
            )

        print_pose("[STATUS]", position, quat)
        print_estimate_comparison()
        time.sleep(0.5)


def land_and_disarm(cf):
    print("[SAFETY] Landing...")
    try:
        cf.high_level_commander.land(LAND_Z, LAND_DURATION)
        time.sleep(LAND_DURATION + 0.5)
        cf.high_level_commander.stop()
    finally:
        print("[SAFETY] Disarming...")
        cf.platform.send_arming_request(False)


def main():
    print("=" * 72)
    print("GUARDED MOCAP TAKEOFF")
    print("=" * 72)
    print(f"Rigid body: {rigid_body_name}@{host_name}")
    print(f"Takeoff target: z={TAKEOFF_Z:.3f}m ({TAKEOFF_HEIGHT_ABOVE_FLOOR:.3f}m above floor)")
    print(f"Bounds: {CAGE_BOUNDS}, safety_margin={SAFETY_MARGIN:.2f}m")
    print("This script only performs a low takeoff, short hover, and landing.")
    print("=" * 72)

    cflib.crtp.init_drivers()
    mocap_wrapper = None
    estimate_logconf = None
    armed = False
    cf = None

    try:
        with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
            cf = scf.cf

            mocap_wrapper = MocapWrapper(rigid_body_name)
            mocap_wrapper.on_pose = lambda x, y, z, quat: send_extpose_quat(cf, x, y, z, quat)

            start_position = wait_for_fresh_pose()
            require_stable_pose()
            is_safe, reason = position_safe(start_position)
            print(f"[INFO] Start position safety: {reason}")
            if not is_safe:
                raise RuntimeError(f"Start position is outside guarded bounds: {reason}")

            input("Press ENTER to arm and run the guarded low takeoff, or Ctrl+C to abort...")

            print("[INFO] Configuring estimator and high-level commander...")
            adjust_orientation_sensitivity(cf)
            activate_kalman_estimator(cf)
            enable_high_level_commander(cf)
            estimate_logconf = setup_estimate_logger(cf)

            print("[INFO] Resetting estimator while external pose is streaming...")
            reset_estimator(cf)
            time.sleep(1.0)
            print_estimate_comparison()

            print("[INFO] Arming...")
            cf.platform.send_arming_request(True)
            armed = True
            time.sleep(1.0)

            monitor_guarded_hover(cf.high_level_commander, start_position)
            print("[FLIGHT] Guarded hover completed")
            land_and_disarm(cf)
            armed = False
            print("[SUCCESS] Guarded takeoff test complete")
    except KeyboardInterrupt:
        print("\n[INTERRUPT] Aborted by user")
        if armed:
            land_and_disarm(cf)
    except Exception as exc:
        print(f"\n[ERROR] {exc}")
        if armed:
            land_and_disarm(cf)
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
