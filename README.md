# Overview
This repo consists of the python files that we use to control the LocoBot. 

Specifically, the ```arm_controller_vertical.py``` contains all the relevant code for controlling the arm during the use of the system with the HoloLens and Workspace Cameras

Please see [PyRobot Read the docs](https://pyrobot-next.readthedocs.io/en/api_0.4/) for information on how to program the robot. The rest is basic ROS communication and robot IK.

# Setup & Getting Running

To run the Arm:
- Source the Environment:
```source ~/pyenv_pyrobot_python2/bin/activate```
- Run the Arm Nodes:
```roslaunch locobot_control main.launch use_arm:=true```

To run our arm controller just run it like any other python file (don't try to rosrun, it will likely break)

```python arm_controller_vertical.py```
