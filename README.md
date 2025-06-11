# Line Following
Hello, this is a simple ROS2 package for line following. The setup is made on:

- Ubuntu 22.04 with ROS2 Humble installed
- Turtlebot3 with ROS2 and a Raspberry Pi camera installed
- The turtlebot runs with a Raspberri Pi 4 4gb. This is the minimum recommended since weaker versions won't work. 
- The tracks from the autorace from Turtlebot3 (but "any line" will do the job, I will explain after)


## Description of the project

This line following code will detect a **yellow** line and follow it by keeping an offset distance to the right of the line, staying on the track.


## Installation

### Method 1:

The most simple way. If you already have a workspace up and running, just clone it on your src folder, colcon build it and source the *install/setup.bash* file.

```
cd your_workspace
colcon build --packages-select line_navigation && source install/setup.bash
```

### Method 2:

If you want you can create a fresh package and just copy and paste the contents, specially the **line_navigation.py** and the **setup.py**

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

The code runs with */image_raw/compressed* topic, so as long as you have this type of image, it will work regardless your camera driver.

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

This one will keep printing the error, speeds and kp parameter, it will also stop the code if you press ```Ctrl + C``` with this window chosen.

A tunning window and the resulted segmentation, used to see if your tunning parameters are well selected and the quality of the detection. <br><hl>
![Screenshot from 2025-02-19 17-57-55](https://github.com/user-attachments/assets/964745a9-a17a-4ef1-9499-01933032cf5e)
![Screenshot from 2025-02-19 17-57-32](https://github.com/user-attachments/assets/c7fb1d55-fdab-471a-9f1e-e9b3648a1e8b)


In the tunning window we have:
- **Low and High Yellow H**: Tune this parameter for the yellow detection.
- **Linear Speed**: As the name suggests, controls the speed of the robot. A value between 4-5 is more than enough.
- **KP**: This value is a constant that will influentiates how much the angular speed will be affected when the robot will turn. Higher values are more adequate if you are following the line on top of it, for the case presented, lower values will work.
- **ROI Height**: It will crop the detection window. Used to avoid unwanted detections.

You will launch your robot after feeling satisfied with this tunning. You can keep changing while the robot is moving, of course.

## Notes on tunning, variables and other comments
- You can adapt for other colors, just change the lines 62 and 63 in the *line_navigation.py* file for detecting what color you want. Just make sure to adapt/rename the other variables depending on this.
- There is a parameter called LEFT_OFFSET (line 15) that makes the robot follow the line at the right side. The more you reduce this value the more on top of the line the robot will be.
- The MIN_AREA_TRACK (line 120) is a variable set for avoid detecting thinner lines. You can play with this value if you need to. When you DECREASE this value, it will detect thinner lines.
- On line 40, there is a timeout variable. If the robot stops detecting the line, it will stop after 5 seconds. This also added some robustness, since it preserves the last valid speed and direction of the robot. As soons as it starts detecting a line again, it will continue the normal behavior.
- All of these values were made for our setup here in the lab. You will definitely need to play with all of these values to achieve a good setup for your case.

## DEMO
A small gif showing the robot moving, the video is too big for github. When the robot is approaching the box by the end of the turn it stops detecting the line for a line, but the timeout variable makes the robot continue!

![ezgif-5c4234361fa7c3](https://github.com/user-attachments/assets/4af3fc61-7b20-41eb-9a07-5bffb74da0a8)


## Thank you!
