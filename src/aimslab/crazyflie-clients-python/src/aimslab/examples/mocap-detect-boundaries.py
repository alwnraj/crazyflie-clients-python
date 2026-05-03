"""
Automatically detect cage boundaries from the mocap system's tracking volume.
This script connects to the mocap system and attempts to determine the safe flight boundaries.
"""
import motioncapture
import time
import json

# Mocap settings
host_name = '192.168.1.42:3883'
mocap_system_type = 'vrpn'
rigid_body_name = 'crazyflie_21'

# Safety margin to add after detecting boundaries
SAFETY_MARGIN = 0.3  # meters

def detect_mocap_boundaries():
    """
    Connect to mocap system and attempt to detect the tracking volume boundaries.
    """
    print("="*60)
    print("AUTOMATIC BOUNDARY DETECTION")
    print("="*60)
    print(f"[INFO] Connecting to mocap system: {mocap_system_type} at {host_name}")
    
    try:
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        print("[SUCCESS] Connected to mocap system")
        
        # Check if the mocap system provides tracking volume info
        print("\n[INFO] Mocap system information:")
        print(f"  Type: {mocap_system_type}")
        
        # Try to get tracking volume if available
        if hasattr(mc, 'trackingVolume'):
            volume = mc.trackingVolume
            print(f"  Tracking volume: {volume}")
        else:
            print("  Tracking volume: Not directly available from API")
        
        # Alternative: Sample positions over time to find extent
        print("\n[INFO] Looking for tracked rigid bodies...")
        
        # Wait for frames and collect rigid body data
        sample_count = 100
        positions = {name: {'x': [], 'y': [], 'z': []} for name in [rigid_body_name]}
        found_bodies = set()
        
        print(f"[INFO] Sampling {sample_count} frames to detect rigid body extents...")
        for i in range(sample_count):
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name not in found_bodies:
                    print(f"[FOUND] Rigid body: {name}")
                    found_bodies.add(name)
                
                if name == rigid_body_name:
                    pos = obj.position
                    positions[name]['x'].append(pos[0])
                    positions[name]['y'].append(pos[1])
                    positions[name]['z'].append(pos[2])
            
            if i % 10 == 0:
                print(f"  Sampled {i}/{sample_count} frames...")
            time.sleep(0.01)
        
        # Analyze collected data
        if rigid_body_name in positions and len(positions[rigid_body_name]['x']) > 0:
            print(f"\n[SUCCESS] Found '{rigid_body_name}' in mocap data")
            print(f"[INFO] Collected {len(positions[rigid_body_name]['x'])} position samples")
            
            # Calculate current position bounds (where drone currently is)
            x_vals = positions[rigid_body_name]['x']
            y_vals = positions[rigid_body_name]['y']
            z_vals = positions[rigid_body_name]['z']
            
            current_x = sum(x_vals) / len(x_vals)
            current_y = sum(y_vals) / len(y_vals)
            current_z = sum(z_vals) / len(z_vals)
            
            print(f"\n[INFO] Current drone position (average):")
            print(f"  X: {current_x:.3f}m")
            print(f"  Y: {current_y:.3f}m")
            print(f"  Z: {current_z:.3f}m")
            
            print(f"\n[INFO] Position variance during sampling:")
            print(f"  X range: [{min(x_vals):.3f}, {max(x_vals):.3f}] (spread: {max(x_vals)-min(x_vals):.3f}m)")
            print(f"  Y range: [{min(y_vals):.3f}, {max(y_vals):.3f}] (spread: {max(y_vals)-min(y_vals):.3f}m)")
            print(f"  Z range: [{min(z_vals):.3f}, {max(z_vals):.3f}] (spread: {max(z_vals)-min(z_vals):.3f}m)")
            
            print("\n" + "="*60)
            print("BOUNDARY DETECTION RESULT")
            print("="*60)
            print("\n⚠️  MANUAL MAPPING REQUIRED")
            print("\nThe mocap system can track the drone anywhere in the cage,")
            print("so boundaries cannot be auto-detected from position data alone.")
            print("\nYou have two options:")
            print("\n1. MANUAL MEASUREMENT (Recommended - Most Accurate)")
            print("   - Measure your cage with a tape measure")
            print("   - Update CAGE_BOUNDS in the boundary-aware script")
            print("\n2. AUTOMATED MAPPING FLIGHT (See next script)")
            print("   - The drone will explore the space")
            print("   - Manually stop it before it hits walls")
            print("   - It will record the boundaries it explored")
            
        else:
            print(f"\n[WARNING] Rigid body '{rigid_body_name}' not found in mocap data")
            print("[INFO] Available rigid bodies:")
            for name in found_bodies:
                print(f"  - {name}")
        
    except Exception as e:
        print(f"\n[ERROR] Failed to detect boundaries: {e}")
        print("\n[SOLUTION] Please manually measure your cage dimensions")
    
    print("\n" + "="*60)


def print_example_config(center_x, center_y, cage_width, cage_depth, cage_height):
    """
    Print example configuration based on manual measurements.
    """
    print("\n[EXAMPLE] If your cage dimensions are:")
    print(f"  - Width (X): {cage_width}m")
    print(f"  - Depth (Y): {cage_depth}m")
    print(f"  - Height (Z): {cage_height}m")
    print(f"  - Center at: ({center_x}, {center_y}, 0)")
    print("\nAdd this to your script:")
    print("\n```python")
    print("CAGE_BOUNDS = {")
    print(f"    'x_min': {center_x - cage_width/2:.2f},")
    print(f"    'x_max': {center_x + cage_width/2:.2f},")
    print(f"    'y_min': {center_y - cage_depth/2:.2f},")
    print(f"    'y_max': {center_y + cage_depth/2:.2f},")
    print(f"    'z_min': 0.0,")
    print(f"    'z_max': {cage_height:.2f},")
    print("}")
    print("SAFETY_MARGIN = 0.3  # 30cm from walls")
    print("```\n")


if __name__ == '__main__':
    detect_mocap_boundaries()
    
    print("\n" + "="*60)
    print("NEXT STEPS")
    print("="*60)
    print("\n1. Measure your cage with a tape measure:")
    print("   - Width (X direction)")
    print("   - Depth (Y direction)")
    print("   - Height (Z direction)")
    print("   - Note the center position (look at mocap coordinate system)")
    
    print("\n2. Run the mapping flight script (if you want automated mapping):")
    print("   python3 mocap-map-boundaries.py")
    
    print("\n3. Update the boundary-aware flight script with your measurements")
    
    # Show example configurations
    print("\n" + "="*60)
    print("COMMON CAGE SIZE EXAMPLES")
    print("="*60)
    
    print("\n--- Example 1: 3m x 3m x 2m cage centered at origin ---")
    print_example_config(0, 0, 3.0, 3.0, 2.0)
    
    print("\n--- Example 2: 2m x 2m x 1.5m cage centered at origin ---")
    print_example_config(0, 0, 2.0, 2.0, 1.5)
    
    print("\n" + "="*60)


