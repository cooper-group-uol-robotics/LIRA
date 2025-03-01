import argparse
import json
import socket
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import time
import math
import select
import numpy as np
from aruco_detector.msg import ArucoPose
import tf.transformations as tft
import threading


def parse_arguments():
    parser = argparse.ArgumentParser(description="Server to interact with KUKA LBR.")
    parser.add_argument('--mode', choices=['record_waypoint', 'calculate_Tmarker', 'send_to_kuka', 'save_eye_hand', 'save_Tbase_to_ee', 'save_current_Tcamera_to_marker', 'save_Tcamera_to_marker_1', 'testing'], required=True,
                        help="Mode in which to run the server.")
    return parser.parse_args()

class KukaServer:
    def __init__(self):
        self.should_stop = False
        self.ip = '172.31.1.10'
        self.port = '113011'
        self.buffer = ""
        self.HOST = '0.0.0.0'  # Listen on all interfaces
        self.PORT = 65432       # Arbitrary port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.bind((self.HOST, self.PORT))
        # self.s.listen()  
        self.saved_poses = {"matrix": []}
        self.s.settimeout(5)   
        rospy.init_node('kuka_listener')
        self.joint_pub = rospy.Publisher('kuka_joint_states', JointState, queue_size=10)
        self.pose_pub = rospy.Publisher('kuka_ee_pose', Pose, queue_size=10)    
        self.marker_pose = None
        self.marker_sub = rospy.Subscriber("/aruco_pose", ArucoPose, self.marker_callback)

    def establish_connection(self):
        self.s.listen()
        self.client, _ = self.s.accept()
    
    def keyboard_listener(self):
        input("Press Enter to stop...\n")
        self.should_stop = True

    def socket_kuka(self, data):
        s_kuka = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s_kuka.connect((self.ip, int(self.port)))  
        s_kuka.sendall(data.encode())
        s_kuka.close()


    def quaternion_to_abc(self, qw, qx, qy, qz):
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        qw /= norm
        qx /= norm
        qy /= norm
        qz /= norm
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        pitch = math.asin(2*(qw*qy - qz*qx))
        roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))

        A = yaw
        B = pitch
        C = roll
        return A, B, C
    
    def invert_transform(self, matrix):
        """Inverts a 4x4 transformation matrix."""
        return np.linalg.inv(matrix)

    def multiply_transforms(self, *matrices):
        """Multiplies multiple 4x4 transformation matrices."""
        result = np.eye(4)
        for matrix in matrices:
            result = np.dot(result, matrix)
        return result
    def quaternion_to_matrix(self, quaternion):

        return tft.quaternion_matrix(quaternion)[:3, :3]

    def marker_callback(self, msg):
        """Callback for the aruco pose subscriber."""
        # Extract translation (position) and quaternion (orientation) from the ArucoPose
        t = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        q = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]

        rotation_matrix = self.quaternion_to_matrix(q)  

        # Create the 4x4 transformation matrix
        self.marker_pose = [
            [rotation_matrix[0][0], rotation_matrix[0][1], rotation_matrix[0][2], t[0]],
            [rotation_matrix[1][0], rotation_matrix[1][1], rotation_matrix[1][2], t[1]],
            [rotation_matrix[2][0], rotation_matrix[2][1], rotation_matrix[2][2], t[2]],
            [0, 0, 0, 1]
        ]

    def save_Tcamera_to_marker_1(self):
        # Wait until we receive a marker pose
        while self.marker_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

            data = {
                "matrix": self.marker_pose
            }

        # Save the received marker pose to a JSON file
        with open("Tcamera_to_marker_1.json", "w") as file:
            json.dump(data, file, indent = 4)

        print("Tcamera_to_marker_1 saved successfully!")

    def save_Tcamera_to_marker_c(self):
        # Wait until we receive a marker pose
        while self.marker_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)

        # Save the received marker pose to a JSON file
        with open("Tcamera_to_marker_c.json", "w") as file:
            json.dump(self.marker_pose, file)

        print("Fetching current Tcamera_to_marker !")

    def format_pose_msg(self, pose_msg):
        x =  pose_msg.position.x
        y =  pose_msg.position.y
        z =  pose_msg.position.z
        rx = pose_msg.orientation.x
        ry = pose_msg.orientation.y
        rz = pose_msg.orientation.z
        w = pose_msg.orientation.w
        
        return [x, y, z, rx, ry, rz, w]
    
    def euler_to_quaternion(self, a, b, c):
        ca = math.cos(a/2)
        sa = math.sin(a/2)
        cb = math.cos(b/2)
        sb = math.sin(b/2)
        cc = math.cos(c/2)
        sc = math.sin(c/2)
        x = ca * cb * sc - sa * sb * cc
        y = ca * sb * cc + sa * cb * sc
        z = sa * cb * cc - ca * sb * sc
        w = ca * cb * cc + sa * sb * sc
        return x, y, z, w
    
    def save_poses_to_json(self):
        with open("ee_poses.json", "w") as file:
            json.dump(self.saved_poses, file, indent = 4)
    
    def load_transformation_from_json(self, filename):
        with open(filename, 'r') as file:
            data = json.load(file)
        return np.array(data["matrix"])

    def save_Tee_to_camera_to_json(self):
        matrix = [
            [0.08294611,  0.87322798, -0.48020083, -0.18513282],
            [-0.99459351,  0.10274895,  0.01504702,  0.02774218],
            [0.06247961,  0.47635654,  0.8770295,   0.07397514],
            [0, 0, 0, 1]
        ]
        data = {
            "matrix": matrix
        }

        with open("Tee_to_camera.json", "w") as json_file:
            json.dump(data, json_file, indent=4)

    def record_waypoint(self):
        threading.Thread(target=self.keyboard_listener).start()
        try:
            while not self.should_stop:

                try:
                    conn, addr = self.s.accept()
                except socket.timeout:
                    continue

                print("Connected by", addr)
                buffer = ""
                while not self.should_stop:
                    try:
                        ready_to_read, _, _ = select.select([conn], [], [], 1.0)
                        if ready_to_read:
                            data = conn.recv(1024)
                        else:
                            continue
              
                        if not data:
                            print("Connection lost. Waiting for a new connection...")
                            break

                        buffer += data.decode('utf-8')

                        if "\n" in buffer:
                            msg = buffer.split('\n')[0]
                            buffer = buffer.split('\n', 1)[1]
                            print("Received data:", msg)
                            if "|" in msg:
                                pose_data, joint_data_string = msg.split('|')
                            # if msg.startswith("JOINTS:"):
                                joints_data = joint_data_string.split(',')
                                # msg.split(":")[1].split(',')
                                joint_msg = JointState()
                                joint_msg.name = ["Joint1", "Joint2", "Joint3", "Joint4", "Joint5", "Joint6", "Joint7"]
                                joint_msg.position = [float(joint.split(":")[1]) for joint in joints_data]
                                print("joint_msg", joint_msg)
                                self.joint_pub.publish(joint_msg)
                            # elif "X:" in msg and "Y:" in msg and "Z:" in msg:

                                pose_data = pose_data.split(",")
                                pose_msg = Pose()
                                pose_msg.position.x = float(pose_data[0].split(":")[1].strip())/1000
                                pose_msg.position.y = float(pose_data[1].split(":")[1].strip())/1000
                                pose_msg.position.z = float(pose_data[2].split(":")[1].strip())/1000
                                A = float(pose_data[3].split(":")[1].strip())
                                B = float(pose_data[4].split(":")[1].strip())
                                C = float(pose_data[5].split(":")[1].strip())

                                qx, qy, qz, qw = self.euler_to_quaternion(A, B, C)
                                # AA, BB, CC = quaternion_to_abc(qw, qx, qy, qz)
                                pose_msg.orientation.x = qx
                                pose_msg.orientation.y = qy
                                pose_msg.orientation.z = qz
                                pose_msg.orientation.w = qw
                                # print("pose_data", pose_data)
                                # print("pose_msg", pose_msg)
                                formatted_pose = self.format_pose_msg(pose_msg)
                                print("pose_msg_formated", formatted_pose)
                                matrix_format = [
                                    [formatted_pose[0], formatted_pose[1], formatted_pose[2], 1.0],
                                    [formatted_pose[3], formatted_pose[4], formatted_pose[5], formatted_pose[6]]
                                ]
                                # self.saved_poses.append({
                                #     "position": [formatted_pose[0], formatted_pose[1], formatted_pose[2]],
                                #     "orientation": [formatted_pose[3], formatted_pose[4], formatted_pose[5], formatted_pose[6]]
                                # })
                                self.saved_poses["matrix"].append(matrix_format)
                                # print(f"A: {AA}, B: {BB}, C: {CC}") # for validation by comparing with smartpad
                                self.pose_pub.publish(pose_msg)

                        time.sleep(0.1)
                        conn.sendall(b"ACK\n")

                    except socket.error as e:
                        print("Socket error", e)
                        break
        finally:
            if 'conn' in locals():
                conn.close()
            self.s.close()
            print("\nSaving poses...")
            self.save_poses_to_json()

    def calculate_Tmarker_to_ee(self):
        # Load saved end effector poses
        with open("ee_poses.json", "r") as file:
            ee_poses = json.load(file)["matrix"]

        # Load Tee_to_camera
        with open("Tee_to_camera.json", "r") as file:
            Tee_to_camera_data = json.load(file)
            Tee_to_camera = np.array(Tee_to_camera_data["matrix"])

        # Load Tcamera_to_marker_1
        with open("Tcamera_to_marker_1.json", "r") as file:
            Tcamera_to_marker_1_data = json.load(file)
            Tcamera_to_marker_1 = np.array(Tcamera_to_marker_1_data["matrix"])

        # Get the first ee pose (Tbase_to_ee_1)
        Tbase_to_ee_1 = np.array(ee_poses[0])

        Tmarker_to_ee = []

        # Calculate Tmarker_to_ee_i for each ee pose
        for Tbase_to_ee_i in ee_poses:
            Tbase_to_ee_i = np.array(Tbase_to_ee_i)
            Tmarker_to_ee_i = self.multiply_transforms(
                self.invert_transform(Tbase_to_ee_i),
                Tbase_to_ee_1,
                Tee_to_camera,
                Tcamera_to_marker_1
            )
            Tmarker_to_ee.append(self.invert_transform(Tmarker_to_ee_i).tolist())

        # Save to JSON
        with open("Tmarker_to_ee.json", "w") as file:
            json.dump({"matrix": Tmarker_to_ee}, file, indent=4)

    def save_Tbase_to_ee(self):
        # The code to send data to KUKA goes here.
        # ideally, i need to send Tbase_to_ee
        # get Tcamera_to_marker_c (c represents current), then 
        # calculate Tbase_to_marker_c = Tbase_to_ee_1 (this is fixed config for checking the marker) * Tee_to_camera * Tcamera_to_marker_c
        # finally calculate Tbase_to_ee = Tbase_to_marker_c * Tmarker_to_ee (saved json from save_Tcamera_to_marker_c)
        
        # Load the saved transformations
        Tee_to_camera = self.load_transformation_from_json('Tee_to_camera.json') # E2H matrix
        Tmarker_to_ee = self.load_transformation_from_json('Tmarker_to_ee.json') # series of ee poses in terms of the marker
        Tcamera_to_marker_c = self.load_transformation_from_json('Tcamera_to_marker_c.json')

        # Load first saved ee pose
        Tbase_to_ee_1 = self.load_transformation_from_json('Tbase_to_ee_1.json')

        # Calculate Tbase_to_marker_c
        Tbase_to_marker_c = self.multiply_transforms(Tbase_to_ee_1, Tee_to_camera, Tcamera_to_marker_c)

        # Calculate final Tbase_to_ee
        Tbase_to_ee = np.matmul(Tbase_to_marker_c, Tmarker_to_ee)
        with open("Tbase_to_ee.json", "w") as file:
            json.dump({"matrix": Tbase_to_ee.tolist()}, file, indent=4)
    
    def send_data_to_kuka(self):
        with open("Tbase_to_ee.json", "r") as file:
            data = json.load(file)["matrix"]
        
        # Extracting orientation (assuming the structure is same as you've mentioned before)
        qw = data['orientation'][3]
        qx = data['orientation'][0]
        qy = data['orientation'][1]
        qz = data['orientation'][2]
        
        # Convert quaternion to Euler angles
        a, b, c = self.quaternion_to_abc(qw, qx, qy, qz)
        
        # Replace quaternion with Euler angles in your data
        data['orientation'] = {'a': a, 'b': b, 'c': c}
        
        # Send this modified data to KUKA
        json_data = json.dumps(data)
        self.client.sendall(json_data)

    def send_data_to_kuka_for_testing(self):
        with open("ee_poses.json", "r") as infile:
            data = json.load(infile)
            first_matrix = data["matrix"][0]
        

        qw = first_matrix[1][3]
        qx = first_matrix[1][0]
        qy = first_matrix[1][1]
        qz = first_matrix[1][2]
        
        # Convert quaternion to Euler angles
        a, b, c = self.quaternion_to_abc(qw, qx, qy, qz)
        
        data_to_send = {
            'x': first_matrix[0][0],
            'y': first_matrix[0][1],
            'z': first_matrix[0][2],
            'a': a,
            'b': b,
            'c': c
        }

        print(data_to_send)
        
        # Send this modified data to KUKA
        # json_data = json.dumps(data_to_send)
        # self.socket_kuka(json_data)

if __name__ == "__main__":
    args = parse_arguments()
    server = KukaServer()
    server.establish_connection()

    if args.mode == "save_Tcamera_to_marker_1":
        server.save_Tcamera_to_marker_1()
    elif args.mode == "record_waypoint":
        server.record_waypoint()
    elif args.mode == "calculate_Tmarker":
        server.calculate_Tmarker_to_ee()
    elif args.mode == "save_current_Tcamera_to_marker":
        server.save_Tcamera_to_marker_c()
    elif args.mode == "save_Tbase_to_ee":
        server.save_Tbase_to_ee()
    elif args.mode == "send_to_kuka":
        server.send_data_to_kuka()
    elif args.mode == "save_eye_hand":
        server.save_Tee_to_camera_to_json()
    elif args.mode == "testing":
        server.send_data_to_kuka_for_testing()
    else:
        print("Unknown mode. Exiting.")