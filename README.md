# Line Following
Hello, this is a simple ROS2 package for line following. The setup is made on:

- Ubuntu 22.04 with ROS Humble installed
- Turtlebot3 with ROS2 and a Raspberry Pi camera installed 
- The tracks from the autorace from Turtlebot3 (but "any line" will do the job, I will explain after)


## Description of the project

This line following code will detect a **yellow** line and follow it by keeping an offset distance to the right of the line, staying on the track.


## Installation

### Method 1:

The most simple. If you already have a workspace up and running, just clone it on your src folder, colcon build it and source the *install/setup.bash* file.

```
cd your_workspace
colcon build --packages-select line_navigation && source install/setup.bash
```

### Method 2:

If you want you can start fresh by creating a fresh package and just copying and pasting the contents, specially the **line_navigation.py** and the **setup.py**

```
cd your_workspace/src
ros2 pkg create --build-type ament_python line_navigation --dependencies rclpy std_msgs geometry_msgs sensor_msgs cv2 cv_bridge
```
Paste the **line_navigation.py** file on the */src/line_navigation/line_navigation/* folder and replace the **setup.py** file on the */src/line_navigation/* folder

Build and source:
```
cd your_workspace/src
colcon build && source install/setup.bash
```

## Running

Connect to the robot and launch the camera
```
ros2 run v4l2_camera v4l2_camera_node
```
The camera package was installed thanks to this tutorial:
https://gitlab.com/boldhearts/ros2_v4l2_camera

The code runs with */image_raw/compressed* topic, so as long as you have this type of image, it will work regardless your camera drive

Then in your workstation launch the line navigation program:
```
ros2 run line_navigation line_navigation_node
```
After choosing your parameters (explained below), connect to the robot again and launch it:
```
ros2 launch turtlebot3_bringup robot.launch.py
```

The robot will start moving.

## Explaning the code:

Once you launch the code, three windows will pop-up:

![Screenshot from 2025-02-19 17-56-43](https://github.com/user-attachments/assets/d8ce5113-9b7c-4006-a387-770f8064b698)

This one will keep printing the error, speed and kp parameter, it will also stop the code if you press Ctrl + C with this window chosen.

A tunning window and the resulted segmentation, used to see if your tunning parameters are well selected and the quality of the detection.
![Screenshot from 2025-02-19 17-57-55](https://github.com/user-attachments/assets/964745a9-a17a-4ef1-9499-01933032cf5e)
![Screenshot from 2025-02-19 17-57-32](https://github.com/user-attachments/assets/c7fb1d55-fdab-471a-9f1e-e9b3648a1e8b)


In the tunning window we have:
- **Low and High Yellow H**: Tune this paramater for the yellow detection.
- **Linear Speed**: As the name suggests, controls the speed of the robot. A value between 4-5 is more than enough.
- **KP**: This value is a constant that will influentiates how much the angular speed will be affected when the robot will turn. This value is more adequate if you are following the line on top of it, for the case presented, lower values will work.
- **ROI Height**: It will crop the detection window. Used to avoid unwanted detections.

You will launch your robot after will feel satisfied with this tunning. You can keep changing while the robot is moving, of course.
