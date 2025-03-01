import json
import rospy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
import numpy as np
from aruco_detector.msg import ArucoPose
import tf.transformations as tft
import threading
import transforms3d.quaternions as quater3d
from spyne import Application, ServiceBase, Unicode, rpc
from spyne.protocol.soap import Soap11
from spyne.server.wsgi import WsgiApplication
from spyne import Float, Integer, Mandatory
from spyne.model.complex import ComplexModel
from spyne.model.primitive import Unicode
from wsgiref.simple_server import make_server
import subprocess
from sensor_msgs.msg import Image
import os
from transformers import AutoProcessor, LlavaOnevisionForConditionalGeneration
import torch
from PIL import Image as PILImage
import cv2

class ImageAnalyzer:

    def __init__(self, model_id, device="cuda:0"):
        original_model_id = "llava-hf/llava-onevision-qwen2-0.5b-ov-hf"
        self.device = device
        print("Initializing model, processor...")
        self.model = LlavaOnevisionForConditionalGeneration.from_pretrained(
            model_id,
            torch_dtype=torch.float16,
            low_cpu_mem_usage=True,
        ).to(0)
        self.processor = AutoProcessor.from_pretrained(original_model_id)
        self.model.to(self.device)

    def analyze(self, image_path, question):
        try:
            image = PILImage.open(image_path)
            conversation = [
                {
                    "role": "user",
                    "content": [
                        {"type": "image"},
                        {"type": "text", "text": question},
                    ],
                },
            ]
            prompt = self.processor.apply_chat_template(conversation, add_generation_prompt=True)
            inputs = self.processor(images=image, text=prompt, return_tensors="pt").to(self.device)
            for key, val in inputs.items():
                if val.dtype == torch.float32: 
                    inputs[key] = val.half()
            output = self.model.generate(**inputs, max_new_tokens=150)
            response = self.processor.decode(output[0], skip_special_tokens=True)
            # response = "test"
            return response
        except Exception as e:
            print(f"Error during analysis: {e}")
            return f"Error: {e}"


class PoseData(ComplexModel):
    x = Float
    y = Float
    z = Float
    a = Float
    b = Float
    c = Float

class MyUpdateMarkerFrameResponse(ComplexModel):
    pose = PoseData

class KukaSOAPService(ServiceBase):

    image_analyzer = None
    image_path = "/home/kuka_service/images/current_image.jpg" 
    image_saved_event = threading.Event()  # Class attribute

    def __init__(self):
        """
        Initializes the KukaSOAPService resources.
        """
        rospy.init_node("kuka_soap_service", anonymous=True)
        if KukaSOAPService.image_analyzer is None:
            print("Initializing ImageAnalyzer...")
            KukaSOAPService.image_analyzer = ImageAnalyzer(model_id="Zhengxue/llava-onevision-0.5b-ov_train3")
            print("ImageAnalyzer initialized.")
        
        self.image_saved_event = threading.Event()


    def _resize_image(self, input_path, output_path, target_size=(640, 480)):
        """Resizes an image to the target dimensions (640x480 by default)."""
        try:
            img = cv2.imread(input_path)
            if img is None:
                raise FileNotFoundError(f"Image not found at {input_path}")
            resized_img = cv2.resize(img, target_size, interpolation=cv2.INTER_AREA)
            cv2.imwrite(output_path, resized_img)
            print(f"Image resized to {target_size} and saved to {output_path}")
        except Exception as e:
            print(f"Failed to resize image: {str(e)}")
            raise  # Propagate the error for handling in DescribeScene

    @rpc(Mandatory(Unicode), _returns=MyUpdateMarkerFrameResponse)
    def UpdateMarkerFrame(ctx, id):
        print('The Marker id is:', id)
        server.save_Tcamera_to_marker_c(id)
        pose_data = server.send_marker_to_kuka_soap(id)
        return MyUpdateMarkerFrameResponse(pose=pose_data)

    @rpc(Unicode, _returns=Unicode)
    def DescribeScene(self, question):
        try:
            KukaSOAPService.image_saved_event.clear()

            print("Creating a temporary subscriber for image capture...")
            temp_subscriber = rospy.Subscriber("/camera/color/image_raw", Image, KukaSOAPService.image_callback)

            # Wait for the image to be saved
            if not KukaSOAPService.image_saved_event.wait(timeout=5):  # Timeout after 5 seconds
                temp_subscriber.unregister()
                raise TimeoutError("Image capture timed out. No image saved.")

            # Unsubscribe after saving the image
            temp_subscriber.unregister()

            # Verify the saved image
            if not os.path.exists(KukaSOAPService.image_path):
                raise FileNotFoundError(f"Image not found at {KukaSOAPService.image_path}.")

            self._resize_image(
                KukaSOAPService.image_path, 
                KukaSOAPService.image_path, 
                target_size=(640, 480)
            )

            print(f"Analyzing the saved image at {KukaSOAPService.image_path}")
            response = KukaSOAPService.image_analyzer.analyze(KukaSOAPService.image_path, question)
            print("Model response:", response)
            return response
        except Exception as e:
            print(f"Error in DescribeScene: {e}")
            return f"Error: {e}"

    @staticmethod
    def image_callback(msg):
        print("jetson_server_SOAP image_callback triggered!")
        try:
            print("Received image message")
            height, width, channels = msg.height, msg.width, 3
            np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape((height, width, channels))

            # Save the image using Pillow
            pil_image = PILImage.fromarray(np_image, mode="RGB")
            pil_image.save(KukaSOAPService.image_path)
            print(f"Image saved to {KukaSOAPService.image_path}")

            # Notify that the image is saved
            KukaSOAPService.image_saved_event.set()
        except Exception as e:
            print(f"Error saving image: {e}")

def initialize_resources():
    if KukaSOAPService.image_analyzer is None:
        print("Global initialization: Setting up ImageAnalyzer...")
        KukaSOAPService.image_analyzer = ImageAnalyzer(model_id="llava-hf/llava-onevision-qwen2-0.5b-ov-hf")
        print("Global initialization: ImageAnalyzer is ready.")



class KukaServer():
    def __init__(self):
        # rospy.init_node('kuka_listener')
        self.marker_pose = {}
        self.marker_subs = {}
        self.marker_sub = rospy.Subscriber("/aruco_pose", ArucoPose, self.marker_callback)


    def describe_scene(self):
        try:
            # Assuming test_jetson_docker.py is in the same directory and returns a description string
            result = subprocess.check_output(['python', 'test_jetson_docker.py'], text=True)
            return result.strip()
        except subprocess.CalledProcessError as e:
            print("Failed to run the scene description script:", e)
            return "Error processing the request"

    def quaternion_to_abc(self, qw, qx, qy, qz):
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        qw /= norm
        qx /= norm
        qy /= norm
        qz /= norm
        yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
        pitch = math.asin(2*(qw*qy - qz*qx))
        roll = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
        # kuka only accepts degree format for rotation data
        A = yaw
        B = pitch
        C = roll
        return A, B, C


    def marker_callback(self, msg, marker_id):
        """Callback for the aruco pose subscriber."""
        # Extract translation (position) and quaternion (orientation) from the ArucoPose

        tx = msg.pose.position.x * 1000
        ty = msg.pose.position.y * 1000
        tz = msg.pose.position.z * 1000


        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        a, b, c = self.quaternion_to_abc(qw, qx, qy, qz)

        self.marker_pose[marker_id] = [tx, ty, tz, a, b, c]

    def save_Tcamera_to_marker_c(self, id):
        # Wait until we receive a marker pose
        topic_name = f"/aruco_pose/id_{id}"
        if topic_name not in self.marker_subs:
            # If this topic is not already subscribed to, create a new subscriber for it
            self.marker_subs[topic_name] = rospy.Subscriber(topic_name, ArucoPose, self.marker_callback, callback_args=id)

        timeout = rospy.Time.now() + rospy.Duration(5)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if id in self.marker_pose:
                break
            rospy.sleep(0.1)

        if id not in self.marker_pose:
            print(f"Failed to get data for marker id {id}")
            return
            
        while self.marker_pose is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            
        data = {
            "matrix": self.marker_pose
        }

        with open("Tcamera_to_marker_c.json", "w") as file:
            json.dump(data, file, indent=1)

        print("Fetching current Tcamera_to_marker !")

    
    def save_Tee_to_camera_to_json(self):
        # calibration result
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



    def send_marker_to_kuka_soap(self, id):
        with open("Tcamera_to_marker_c.json", "r") as file:
            all_marker_data = json.load(file)["matrix"]

        if id not in all_marker_data:
            print(f"No data found for marker id {id}")
            return None 

        marker_data = all_marker_data[id]
        print('data to send', marker_data)
        tx, ty, tz, a, b, c = marker_data
        return PoseData(x=tx, y=ty, z=tz, a=a, b=b, c=c)


if __name__ == "__main__":
    rospy.init_node("kuka_soap_service", anonymous=True)
    initialize_resources()

    application = Application([KukaSOAPService], 'acl.kuka.soap',
                          in_protocol=Soap11(validator='lxml'),
                          out_protocol=Soap11())

    wsgi_app = WsgiApplication(application)

    server = KukaServer()
    soap_server = make_server('0.0.0.0', 65432, wsgi_app)
    print("Listening to http://0.0.0.0:65432")
    print("Press Ctrl+C to stop")
    soap_server.serve_forever()
