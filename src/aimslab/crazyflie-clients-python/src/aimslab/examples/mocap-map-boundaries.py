"""
Interactive boundary mapping flight.
The drone will slowly move in each direction while you observe.
Stop the movement (press Ctrl+C) when it gets close to a wall, and it will record that boundary.
After mapping all boundaries, it saves a configuration file.
"""
import time
import motioncapture
import cflib.crtp
import json
from threading import Thread
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie
uri = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E7E7')

# Mocap settings
host_name = '192.168.1.42:3883'
mocap_system_type = 'vrpn'
rigid_body_name = 'crazyflie_21'
orientation_std_dev = 8.0e-3

# Mapping parameters
MAPPING_HEIGHT = 0.8  # meters - height to fly during mapping
MAPPING_SPEED = 0.15  # m/s - slow speed for safety
SAFETY_MARGIN = 0.3  # meters - margin to add to detected boundaries

# Detected boundaries (will be populated during mapping)
detected_bounds = {
    'x_min': None,
    'x_max': None,
    'y_min': None,
    'y_max': None,
    'z_min': 0.0,  # Floor is always 0
    'z_max': None,
}

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
        print(f"[INFO] Mocap connected, tracking '{self.body_name}'")
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    pos = obj.position
                    current_position['x'] = pos[0]
                    current_position['y'] = pos[1]
                    current_position['z'] = pos[2]
                    if self.on_pose:
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])


def send_extpose_quat(cf, x, y, z, quat):
    cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)


def adjust_orientation_sensitivity(cf):
    cf.param.set_value('locSrv.extQuatStdDev', orientation_std_dev)


def activate_kalman_estimator(cf):
    cf.param.set_value('stabilizer.estimator', '2')


def enable_high_level_commander(cf):
    cf.param.set_value('commander.enHighLevel', '1')


def move_direction_until_stopped(cf, direction, description):
    """
    Move the drone slowly in a direction until user stops it.
    Returns the position where it was stopped.
    """
    commander = cf.high_level_commander
    
    print(f"\n{'='*60}")
    print(f"MAPPING: {description}")
    print(f"{'='*60}")
    print("The drone will move slowly in this direction.")
    print("⚠️  WATCH THE DRONE CAREFULLY!")
    print("⚠️  Press Ctrl+C when it gets about 30-50cm from the wall")
    print(f"{'='*60}\n")
    
    input("Press ENTER when ready to start moving...")
    
    # Record starting position
    start_x = current_position['x']
    start_y = current_position['y']
    start_z = current_position['z']
    
    print(f"[START] Position: ({start_x:.2f}, {start_y:.2f}, {start_z:.2f})")
    print("[INFO] Moving... (press Ctrl+C to stop)")
    
    try:
        # Move continuously in the specified direction
        step_distance = 0.1  # meters per step
        step_time = step_distance / MAPPING_SPEED
        
        while True:
            # Calculate next waypoint
            if direction == 'x+':
                target_x = current_position['x'] + step_distance
                target_y = current_position['y']
                target_z = current_position['z']
            elif direction == 'x-':
                target_x = current_position['x'] - step_distance
                target_y = current_position['y']
                target_z = current_position['z']
            elif direction == 'y+':
                target_x = current_position['x']
                target_y = current_position['y'] + step_distance
                target_z = current_position['z']
            elif direction == 'y-':
                target_x = current_position['x']
                target_y = current_position['y'] - step_distance
                target_z = current_position['z']
            elif direction == 'z+':
                target_x = current_position['x']
                target_y = current_position['y']
                target_z = current_position['z'] + step_distance
            else:  # z-
                target_x = current_position['x']
                target_y = current_position['y']
                target_z = current_position['z'] - step_distance
            
            # Send movement command
            commander.go_to(target_x, target_y, target_z, 0, step_time, relative=False)
            time.sleep(step_time)
            
            # Print current position
            print(f"  Position: ({current_position['x']:.2f}, {current_position['y']:.2f}, "
                  f"{current_position['z']:.2f})", end='\r')
    
    except KeyboardInterrupt:
        print("\n[STOPPED] Movement stopped by user")
        commander.stop()
        time.sleep(0.5)
    
    # Record final position
    final_x = current_position['x']
    final_y = current_position['y']
    final_z = current_position['z']
    
    print(f"\n[FINAL] Position: ({final_x:.2f}, {final_y:.2f}, {final_z:.2f})")
    
    # Return to center
    print("[INFO] Returning to center position...")
    commander.go_to(start_x, start_y, start_z, 0, 3.0, relative=False)
    time.sleep(3.5)
    
    return final_x, final_y, final_z


def save_boundary_config(bounds):
    """Save detected boundaries to a JSON file."""
    config = {
        'cage_bounds': bounds,
        'safety_margin': SAFETY_MARGIN,
        'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
        'notes': 'Auto-generated by boundary mapping flight'
    }
    
    filename = 'cage_boundaries.json'
    with open(filename, 'w') as f:
        json.dump(config, f, indent=2)
    
    print(f"\n[SAVED] Boundary configuration saved to: {filename}")
    return filename


def print_python_config(bounds):
    """Print Python code to copy into flight script."""
    print("\n" + "="*60)
    print("PYTHON CONFIGURATION")
    print("="*60)
    print("\nCopy this into your boundary-aware flight script:")
    print("\n```python")
    print("CAGE_BOUNDS = {")
    for key, value in bounds.items():
        if value is not None:
            print(f"    '{key}': {value:.2f},")
        else:
            print(f"    '{key}': None,  # Not measured")
    print("}")
    print(f"SAFETY_MARGIN = {SAFETY_MARGIN}")
    print("```\n")


def interactive_mapping_flight(cf):
    """
    Guide the user through mapping all boundaries.
    """
    commander = cf.high_level_commander
    
    print("\n" + "="*60)
    print("INTERACTIVE BOUNDARY MAPPING")
    print("="*60)
    print("\nThis script will help you map your cage boundaries.")
    print("The drone will move slowly in each direction.")
    print("YOU must stop it (Ctrl+C) when it gets close to a wall.")
    print("\n⚠️  IMPORTANT SAFETY NOTES:")
    print("  - Stay ready to press Ctrl+C at all times")
    print("  - Stop the drone 30-50cm BEFORE it hits the wall")
    print("  - Keep your emergency stop/power off ready")
    print("  - If anything goes wrong, press Ctrl+C immediately")
    print("="*60 + "\n")
    
    input("Press ENTER to begin mapping (or Ctrl+C to cancel)...")
    
    # Takeoff
    print(f"\n[TAKEOFF] Rising to {MAPPING_HEIGHT}m...")
    commander.takeoff(MAPPING_HEIGHT, 3.0)
    time.sleep(3.5)
    
    # Record center position
    center_x = current_position['x']
    center_y = current_position['y']
    center_z = current_position['z']
    
    print(f"\n[CENTER] Starting position: ({center_x:.2f}, {center_y:.2f}, {center_z:.2f})")
    print("[INFO] This will be used as the center point for mapping")
    
    # Map each boundary
    try:
        # X+ (positive X direction)
        final_x, _, _ = move_direction_until_stopped(cf, 'x+', "Moving in +X direction (right)")
        detected_bounds['x_max'] = final_x - SAFETY_MARGIN
        
        # X- (negative X direction)
        final_x, _, _ = move_direction_until_stopped(cf, 'x-', "Moving in -X direction (left)")
        detected_bounds['x_min'] = final_x + SAFETY_MARGIN
        
        # Y+ (positive Y direction)
        _, final_y, _ = move_direction_until_stopped(cf, 'y+', "Moving in +Y direction (forward)")
        detected_bounds['y_max'] = final_y - SAFETY_MARGIN
        
        # Y- (negative Y direction)
        _, final_y, _ = move_direction_until_stopped(cf, 'y-', "Moving in -Y direction (backward)")
        detected_bounds['y_min'] = final_y + SAFETY_MARGIN
        
        # Z+ (upward) - optional
        print("\n[INFO] Would you like to map the ceiling height? (y/n)")
        if input().lower() == 'y':
            _, _, final_z = move_direction_until_stopped(cf, 'z+', "Moving UP toward ceiling")
            detected_bounds['z_max'] = final_z - SAFETY_MARGIN
        else:
            print("[SKIP] Ceiling height not mapped. Please set manually.")
        
        print("\n" + "="*60)
        print("MAPPING COMPLETE!")
        print("="*60)
        print("\nDetected boundaries (with safety margin applied):")
        for key, value in detected_bounds.items():
            if value is not None:
                print(f"  {key}: {value:.2f}m")
            else:
                print(f"  {key}: Not measured")
        
        # Calculate cage dimensions
        if detected_bounds['x_min'] and detected_bounds['x_max']:
            width = detected_bounds['x_max'] - detected_bounds['x_min']
            print(f"\nCage width (X): {width:.2f}m")
        if detected_bounds['y_min'] and detected_bounds['y_max']:
            depth = detected_bounds['y_max'] - detected_bounds['y_min']
            print(f"Cage depth (Y): {depth:.2f}m")
        if detected_bounds['z_max']:
            print(f"Cage height (Z): {detected_bounds['z_max']:.2f}m")
        
        # Save configuration
        save_boundary_config(detected_bounds)
        print_python_config(detected_bounds)
        
    except Exception as e:
        print(f"\n[ERROR] Mapping failed: {e}")
    
    # Land
    print("\n[LANDING] Bringing drone down...")
    commander.land(0.0, 2.0)
    time.sleep(2.5)
    commander.stop()


def main():
    cflib.crtp.init_drivers()

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        
        # Connect to mocap
        print("[INFO] Connecting to mocap system...")
        mocap_wrapper = MocapWrapper(rigid_body_name)
        mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])
        time.sleep(2.0)
        
        # Configure Crazyflie
        print("[INFO] Configuring estimator...")
        adjust_orientation_sensitivity(cf)
        activate_kalman_estimator(cf)
        enable_high_level_commander(cf)
        reset_estimator(cf)
        time.sleep(2.0)
        
        # Arm
        print("[INFO] Arming drone...")
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        
        try:
            interactive_mapping_flight(cf)
        except KeyboardInterrupt:
            print("\n[INTERRUPT] Mapping cancelled")
            cf.high_level_commander.land(0.0, 2.0)
            time.sleep(2.5)
        finally:
            cf.platform.send_arming_request(False)
            mocap_wrapper.close()
            print("[INFO] Shutdown complete")


if __name__ == '__main__':
    main()

