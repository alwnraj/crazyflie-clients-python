"""
Boundary-aware Crazyflie flight using mocap system.
The drone will fly around the perimeter of a rectangular cage while maintaining
a safe distance from the borders.
"""
import time
from threading import Thread
import motioncapture
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie to connect to
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Mocap settings
host_name = '192.168.1.42:3883'
mocap_system_type = 'vrpn'
rigid_body_name = 'crazyflie_21'
send_full_pose = True
orientation_std_dev = 8.0e-3

# ========== CAGE BOUNDARY CONFIGURATION ==========
# Define your rectangular cage dimensions (in meters)
# Adjust these to match your actual cage size
CAGE_BOUNDS = {
    'x_min': -1.5,  # meters
    'x_max': 1.5,   # meters
    'y_min': -1.5,  # meters
    'y_max': 1.5,   # meters
    'z_min': 0.0,   # meters (floor)
    'z_max': 2.0,   # meters (ceiling)
}

# Safety margin - how far from the walls to stay (in meters)
SAFETY_MARGIN = 0.3  # 30cm from walls

# Flight parameters
FLIGHT_HEIGHT = 0.8  # meters above ground
CORNER_PAUSE_TIME = 1.0  # seconds to pause at each corner
FLIGHT_SPEED = 0.3  # m/s

# Current position tracking (for boundary checking)
current_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}

class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)
        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True
        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print(f"[INFO] Mocap connected, looking for '{self.body_name}'")
        found_body = False
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if not found_body:
                        print(f"[INFO] Found and tracking rigid body: {name}")
                        found_body = True
                    if self.on_pose:
                        pos = obj.position
                        # Update global position for boundary checking
                        current_position['x'] = pos[0]
                        current_position['y'] = pos[1]
                        current_position['z'] = pos[2]
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])


def send_extpose_quat(cf, x, y, z, quat):
    """Send position and orientation to the Crazyflie."""
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


def check_position_safe(x, y, z):
    """
    Check if a position is within safe boundaries.
    Returns (is_safe, reason)
    """
    if x < CAGE_BOUNDS['x_min'] + SAFETY_MARGIN:
        return False, f"Too close to X min boundary ({x:.2f}m)"
    if x > CAGE_BOUNDS['x_max'] - SAFETY_MARGIN:
        return False, f"Too close to X max boundary ({x:.2f}m)"
    if y < CAGE_BOUNDS['y_min'] + SAFETY_MARGIN:
        return False, f"Too close to Y min boundary ({y:.2f}m)"
    if y > CAGE_BOUNDS['y_max'] - SAFETY_MARGIN:
        return False, f"Too close to Y max boundary ({y:.2f}m)"
    if z < CAGE_BOUNDS['z_min'] + SAFETY_MARGIN:
        return False, f"Too close to floor ({z:.2f}m)"
    if z > CAGE_BOUNDS['z_max'] - SAFETY_MARGIN:
        return False, f"Too close to ceiling ({z:.2f}m)"
    return True, "Safe"


def generate_perimeter_waypoints():
    """
    Generate waypoints for flying around the perimeter of the cage.
    Returns a list of (x, y, z) tuples representing corners of the safe flight area.
    """
    # Calculate safe flight boundaries
    x_min_safe = CAGE_BOUNDS['x_min'] + SAFETY_MARGIN
    x_max_safe = CAGE_BOUNDS['x_max'] - SAFETY_MARGIN
    y_min_safe = CAGE_BOUNDS['y_min'] + SAFETY_MARGIN
    y_max_safe = CAGE_BOUNDS['y_max'] - SAFETY_MARGIN
    z_flight = FLIGHT_HEIGHT
    
    # Create rectangular path (counterclockwise when viewed from above)
    waypoints = [
        (x_min_safe, y_min_safe, z_flight),  # Bottom-left corner
        (x_max_safe, y_min_safe, z_flight),  # Bottom-right corner
        (x_max_safe, y_max_safe, z_flight),  # Top-right corner
        (x_min_safe, y_max_safe, z_flight),  # Top-left corner
    ]
    
    return waypoints


def fly_perimeter_path(cf):
    """
    Fly the drone around the perimeter of the cage.
    Uses high-level commander for smooth navigation.
    """
    commander = cf.high_level_commander
    
    print("\n" + "="*60)
    print("BOUNDARY-AWARE PERIMETER FLIGHT")
    print("="*60)
    print(f"Cage bounds: X[{CAGE_BOUNDS['x_min']}, {CAGE_BOUNDS['x_max']}] "
          f"Y[{CAGE_BOUNDS['y_min']}, {CAGE_BOUNDS['y_max']}] "
          f"Z[{CAGE_BOUNDS['z_min']}, {CAGE_BOUNDS['z_max']}]")
    print(f"Safety margin: {SAFETY_MARGIN}m")
    print(f"Flight height: {FLIGHT_HEIGHT}m")
    print("="*60 + "\n")
    
    # Generate safe waypoints
    waypoints = generate_perimeter_waypoints()
    
    print(f"[INFO] Generated {len(waypoints)} perimeter waypoints:")
    for i, (x, y, z) in enumerate(waypoints):
        is_safe, reason = check_position_safe(x, y, z)
        status = "✓ SAFE" if is_safe else f"✗ UNSAFE: {reason}"
        print(f"  Corner {i+1}: ({x:.2f}, {y:.2f}, {z:.2f}) - {status}")
    
    # Takeoff
    print(f"\n[FLIGHT] Taking off to {FLIGHT_HEIGHT}m...")
    commander.takeoff(FLIGHT_HEIGHT, 3.0)
    time.sleep(3.5)
    
    # Check if takeoff position is safe
    is_safe, reason = check_position_safe(
        current_position['x'], 
        current_position['y'], 
        current_position['z']
    )
    print(f"[STATUS] Current position: ({current_position['x']:.2f}, "
          f"{current_position['y']:.2f}, {current_position['z']:.2f}) - {reason}")
    
    if not is_safe:
        print(f"[WARNING] Current position unsafe! {reason}")
        print("[SAFETY] Performing emergency land...")
        commander.land(0.0, 2.0)
        time.sleep(2.5)
        return
    
    # Fly to each corner
    print("\n[FLIGHT] Starting perimeter tour...")
    for i, (x, y, z) in enumerate(waypoints):
        print(f"\n[FLIGHT] Flying to corner {i+1}/{len(waypoints)}: ({x:.2f}, {y:.2f}, {z:.2f})")
        
        # Pre-flight safety check
        is_safe, reason = check_position_safe(x, y, z)
        if not is_safe:
            print(f"[ERROR] Target waypoint unsafe! {reason}")
            print("[SAFETY] Aborting and landing...")
            break
        
        # Calculate distance for timing
        dx = x - current_position['x']
        dy = y - current_position['y']
        dz = z - current_position['z']
        distance = (dx**2 + dy**2 + dz**2)**0.5
        flight_time = distance / FLIGHT_SPEED + 1.0  # Add 1s buffer
        
        # Send goto command
        commander.go_to(x, y, z, 0, flight_time, relative=False)
        
        # Monitor during flight
        time.sleep(flight_time)
        
        # Post-flight position check
        print(f"[STATUS] Reached position: ({current_position['x']:.2f}, "
              f"{current_position['y']:.2f}, {current_position['z']:.2f})")
        
        # Pause at corner
        print(f"[FLIGHT] Pausing at corner for {CORNER_PAUSE_TIME}s...")
        time.sleep(CORNER_PAUSE_TIME)
    
    # Return to start and land
    print("\n[FLIGHT] Perimeter tour complete! Returning to start...")
    start_x, start_y, start_z = waypoints[0]
    commander.go_to(start_x, start_y, start_z, 0, 3.0, relative=False)
    time.sleep(3.5)
    
    print("[FLIGHT] Landing...")
    commander.land(0.0, 2.0)
    time.sleep(2.5)
    
    commander.stop()
    print("\n[SUCCESS] Flight complete! Drone safely landed.\n")


def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        # Connect to mocap system
        print("[INFO] Connecting to mocap system...")
        mocap_wrapper = MocapWrapper(rigid_body_name)
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])
        
        # Wait for mocap to get initial position
        print("[INFO] Waiting for initial position data...")
        time.sleep(2.0)
        
        # Configure Crazyflie
        print("[INFO] Configuring Crazyflie estimator...")
        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        enable_high_level_commander(cf)
        reset_estimator(cf)
        
        print("[INFO] Waiting for estimator to converge...")
        time.sleep(2.0)
        
        # Arm the Crazyflie
        print("[INFO] Arming drone...")
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        
        try:
            # Execute boundary-aware perimeter flight
            fly_perimeter_path(cf)
            
        except KeyboardInterrupt:
            print("\n[INTERRUPT] Keyboard interrupt detected!")
            print("[SAFETY] Emergency landing...")
            cf.high_level_commander.land(0.0, 2.0)
            time.sleep(2.5)
        except Exception as e:
            print(f"\n[ERROR] Exception during flight: {e}")
            print("[SAFETY] Emergency landing...")
            cf.high_level_commander.land(0.0, 2.0)
            time.sleep(2.5)
        finally:
            # Disarm
            print("[INFO] Disarming drone...")
            cf.platform.send_arming_request(False)
            
            # Close mocap connection
            print("[INFO] Closing mocap connection...")
            mocap_wrapper.close()
            
            print("[INFO] Shutdown complete.")


if __name__ == '__main__':
    main()

