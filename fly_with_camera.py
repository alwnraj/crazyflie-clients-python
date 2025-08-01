"""
Flight script with external camera integration.
This script works with your Flow v2 deck and can integrate with external cameras.
"""
import logging
import time
import cv2
import numpy as np
import threading
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
import cflib.crtp

# Use the exact URI that works with your GUI
URI = 'radio://0/80/2M'

# Only output errors from the logging framework
logging.basicConfig(level=logging.ERROR)

class CrazyflieCameraController:
    def __init__(self, uri='radio://0/80/2M', camera_index=0):
        self.uri = uri
        self.camera_index = camera_index
        self.camera = None
        self.camera_thread = None
        self.running = False
        self.latest_frame = None
        
    def start_camera(self):
        """Start external camera (USB camera connected to computer)"""
        try:
            self.camera = cv2.VideoCapture(self.camera_index)
            if not self.camera.isOpened():
                print("⚠ Could not open camera. Continuing without camera...")
                return False
                
            print("✓ External camera started")
            self.running = True
            self.camera_thread = threading.Thread(target=self._camera_loop)
            self.camera_thread.start()
            return True
        except Exception as e:
            print(f"⚠ Camera error: {e}. Continuing without camera...")
            return False
    
    def _camera_loop(self):
        """Camera capture loop running in separate thread"""
        while self.running:
            if self.camera and self.camera.isOpened():
                ret, frame = self.camera.read()
                if ret:
                    self.latest_frame = frame
            time.sleep(0.033)  # ~30 FPS
    
    def get_frame(self):
        """Get the latest camera frame"""
        return self.latest_frame.copy() if self.latest_frame is not None else None
    
    def process_frame(self, frame):
        """Process camera frame for computer vision"""
        if frame is None:
            return None
            
        # Convert to grayscale for processing
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Example: Detect edges
        edges = cv2.Canny(gray, 50, 150)
        
        # Example: Detect circles (could be used for target detection)
        circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 20,
                                  param1=50, param2=30, minRadius=0, maxRadius=0)
        
        return {
            'original': frame,
            'edges': edges,
            'circles': circles
        }
    
    def stop_camera(self):
        """Stop camera and cleanup"""
        self.running = False
        if self.camera_thread:
            self.camera_thread.join()
        if self.camera:
            self.camera.release()
        cv2.destroyAllWindows()
    
    def fly_with_camera_feedback(self):
        """Main flight function with camera integration"""
        # Initialize the low-level drivers
        cflib.crtp.init_drivers(enable_debug_driver=False)
        
        print(f"Connecting to Crazyflie at: {self.uri}")
        
        try:
            with SyncCrazyflie(self.uri) as scf:
                print("✓ Connected successfully!")
                
                # Check if we have a positioning deck
                try:
                    flow_deck = scf.cf.param.get_value("deck.bcFlow2")
                    if flow_deck == '1':
                        print("✓ Flow v2 deck detected - position control available")
                    else:
                        print("⚠ No positioning deck detected - flight may be unstable")
                except:
                    print("⚠ Could not check for positioning deck")

                # Arm the Crazyflie
                print("Arming Crazyflie...")
                scf.cf.platform.send_arming_request(True)
                time.sleep(1.0)

                # Start camera if available
                camera_available = self.start_camera()
                
                # We take off when the commander is created
                with MotionCommander(scf) as mc:
                    print('Taking off!')
                    time.sleep(1)

                    # Flight sequence with camera feedback
                    print('Moving forward 0.5m')
                    mc.forward(0.5)
                    
                    # Process camera data during flight
                    if camera_available:
                        frame = self.get_frame()
                        if frame is not None:
                            processed = self.process_frame(frame)
                            print("✓ Camera data processed during flight")
                    
                    time.sleep(1)

                    print('Moving up 0.2m')
                    mc.up(0.2)
                    time.sleep(1)

                    print('Doing a 270deg circle')
                    mc.circle_right(0.5, velocity=0.5, angle_degrees=270)

                    print('Moving down 0.2m')
                    mc.down(0.2)
                    time.sleep(1)

                    print('Rolling left 0.2m at 0.6m/s')
                    mc.left(0.2, velocity=0.6)
                    time.sleep(1)

                    print('Moving forward 0.5m')
                    mc.forward(0.5)

                    # We land when the MotionCommander goes out of scope
                    print('Landing!')
                    
        except Exception as e:
            print(f"✗ Flight failed: {e}")
        finally:
            # Cleanup camera
            self.stop_camera()

if __name__ == '__main__':
    # Create controller and start flight
    controller = CrazyflieCameraController(URI)
    controller.fly_with_camera_feedback() 