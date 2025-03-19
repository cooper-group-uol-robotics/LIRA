# LIRA
LIRA: Localization, Inspection, and Reasoning Module for Autonomous Workflows in Self-Driving Labs.
LIRA  is an edge computing module that enhances robotic decision-making through VLMs. LIRA enables precise visual localization, automated error inspection, and reasoning, thus allowing robots to adapt dynamically to variations from the expected workflow state.

# Hardware:
Robot: KUKA KMR iiwa, Camera: Intel Realsense D435i, Edge module: NVIDIA AGX Orin(64gb)

# 1.Required environment:
clone the docker image and activate it:
```bash
docker pull zhengxuez/llm_ros
```
# 2.git clone this repository:
```bash
git clone https://github.com/cooper-group-uol-robotics/LIRA.git
cd kuka_server
catkin build
```
# 3.launch the jetson_launch file
```bash
roslaunch jetson_launch jetson_launch.launch
```

# 4.Test with kuka robot
import codes under support_data/client/java_client foler into your SUNRise workspace.
execute InspectionTest.java

# 5.To test the VLM inspection function:
```bash
python support_data/inference.py
```
note: remember to update the path of test_data.json
