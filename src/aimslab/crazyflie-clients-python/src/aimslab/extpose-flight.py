'''Script to receive the UDP stream from Motive VM and sends it to the crazyflie. Keeps the crazyflie external pose updated'''
import threading
import struct
import socket 
import motioncapture
import time
from scipy.spatial.transform import Rotation as R
import numpy as np
import cflib.crtp
from cflib.crazyflie.localization import Localization
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie import Crazyflie
from cflib.crtp.crtpstack import CRTPPacket
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.reset_estimator import reset_estimator

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M/E7E7E7E7E7'
host_ip = "127.0.0.1" # Ip used if receiving data stream from Ricky UDP stream
orientation_std_dev = 8.0e-3 #Std dev for orientation sensitivity
rigid_body_name = "crazyflie_21" #Rigid body name in motive (updated to match your mocap system)
using_mocap = True #Uses VRPN class if true, else it uses UDP stream 

CRTP_PORT_LOCALIZATION = 6
EXT_POSE = 8
GENERIC_TYPE = 1



class MocapWrapper(threading.Thread):
    def __init__(self, body_name):
        threading.Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True

        self.start()

    def close(self):
        self._stay_open = False

    def run(self):
        mc = motioncapture.connect("vrpn", {"hostname": "192.168.1.42"})
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
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])

def activate_kalman_estimator(cf):
    cf.param.set_value("stabilizer.estimator", "2")

def adjust_orientation_sensitivity(cf, orient_std_dev):
    cf.param.set_value("locSrv.extQuatStdDev", orient_std_dev)

def simple_takeoff(scf, cf_local):
    with MotionCommander(scf, default_height = .5) as mc:
        try:
            mc.stop()
            mc.land()
        except KeyboardInterrupt:
            print("[ERROR] Received Keyboard Interrupt...")
            cf_local.send_emergency_stop()

def add_logs(scf, groupName, paramaterdict: dict):
    """Takes in a dictionary of the paramater(groupname.value) and its dtype(float, int, etc.)"""
    logconf = LogConfig(name=groupName, period_in_ms=10)
    for key, value in paramaterdict.items():
            logconf.add_variable(key, value)
    scf.cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_pos_callback)
    logconf.start()
    return logconf

def send_extpose_udp(cf_local, sock):
    """Sends crazyflie pose data using its own API  and unpacking """
    try:
        while True:
            response, ipAddr = sock.recvfrom(1024)#Receives UDP packet

            if len(response) == 29:
                qx, qy, qz, qw, x, y, z = struct.unpack("<7f", response[0:28])
                # qx, qy, qz, qw = rotate_quat(qx, qy, qz, qw)
                cf_local.send_extpose([x, y, z], [qx, qy, qz, qw])#Motive uses y as vertical axis, so we flip z and y

            time.sleep(0.005)#Sleep for crazyflie to handle stream

    except sock.timeout: 
        print("[DEBUG] Socket Timed out. Performing Emergency Landing")
        cf_local.send_emergency_stop()

# Counter for occasional debug output
_pose_counter = [0]  # Use list to make it mutable in nested function

def send_extpose_vrpn(cf, x, y, z, quat):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    
    # Print pose data every 100 frames to verify data flow
    _pose_counter[0] += 1
    if _pose_counter[0] % 100 == 0:
        print(f"[DEBUG] Sending pose: Pos({x:.2f}, {y:.2f}, {z:.2f}) Quat({quat.w:.2f}, {quat.x:.2f}, {quat.y:.2f}, {quat.z:.2f})")

def rotate_quat(qx, qy, qz, qw):
    """Rotates the received orientation by -90 to adjust for frame difference (Motive: x,z, y | Crazyflie: x, y, z)"""
    # Fixed rotation from Y-up (Motive) to Z-up (Crazyflie ENU)
    q_rot = R.from_euler("x", -90, degrees=True)

    q_motive = R.from_quat([qx, qy, qz, qw])
    q_cf = q_rot * q_motive
    return q_cf.as_quat()

#Logging the received pose info from crazyflie
def log_pos_callback(timestamp, data, logconf):
    print("Timestamp: ", timestamp)
    print(data)

#Function to decode packet into CRTP format(Not necessary if sending with ExtPose class)
def decodePacket(packet_recv):
    pk = CRTPPacket()
    pk.port = CRTP_PORT_LOCALIZATION
    pk.channel = GENERIC_TYPE
    pk.data = bytearray()
    pk.data.append(EXT_POSE)
    pk.data += packet_recv
    return pk


def main():
    cflib.crtp.init_drivers()

   # Register for all rigid-body updates
    print("[DEBUG] Connecting to Crazyflie..")

    # Syncs to the crazyflie and establishes our connection
    with SyncCrazyflie(uri, Crazyflie(rw_cache="./cache")) as scf:
        print("[DEBUG] Connected to Crazyflie..")
        adjust_orientation_sensitivity(scf.cf, orientation_std_dev)#
        activate_kalman_estimator(scf.cf)# Tells kalman filter to use extpose
        cf_local = Localization(crazyflie = scf.cf)

        if using_mocap: 
            # Connect to the mocap system
            print(f"[INFO] Connecting to mocap system and looking for rigid body: '{rigid_body_name}'")
            mocap_wrapper = MocapWrapper(rigid_body_name)
            # Set up a callback to handle data from the mocap system
            mocap_wrapper.on_pose = lambda pose: send_extpose_vrpn(scf.cf, pose[0], pose[1], pose[2], pose[3])
        else:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)#create UDP socket 
            sock.bind((host_ip, 10444)) #Bind socket to our specified port
            thread = threading.Thread(target=send_extpose_udp, args=(cf_local, sock), daemon=True)
            thread.start()
 
        #asking for crazyflie to send us pose data on callback
        # positionLog = add_logs(scf, "Position", { 
        #                            "stateEstimate.x" : "float",
        #                            "stateEstimate.y" : "float", 
        #                             "stateEstimate.z" : "float" })
        #Logs the roll pitch and yaw
        # stabilizerLog = add_logs(scf, "Kalman", {
        #                         "kalman.q0": "float",
        #                         "kalman.q1": "float",
        #                         "kalman.q2": "float",
        #                         "kalman.q3": "float"})

        #Allows kf to converge with extpose
        reset_estimator(scf.cf)



        #Arms the drone
        scf.cf.platform.send_arming_request(True)

        print("Taking Off")

        simple_takeoff(scf, cf_local)
        
        #Disarms the drone
        scf.cf.platform.send_arming_request(False)

        if using_mocap:
            mocap_wrapper.close()

if __name__ == "__main__":
    main()