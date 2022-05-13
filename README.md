# Robots_and_Autonomous_Systems
RAS Team project 2022 spring semester

- Sumit Gore <sumit.n.gore@utu.fi>,
- Marius Gurgu  <marius-mihail.m.gurgu@utu.fi>,
- Myrthe Tilleman  <myrthe.e.tilleman@utu.fi>,
- Bowen Tan  <bowen.b.tan@utu.fi>


# Instructions on how to run the code

## Notes
We ran this code on two seperate machines, one that ran the Tello code and one that ran Jetbot code and path tracking algorithm. They were both connected to the same network.

## Pre-requisites
There are many things that are required before being able to run our code. We assume in these instructions that if they are not described, you know how to find, install and run the following:

- ROS Noetic
- ROS Foxy
- Optitrack System
- ROS2-to-ROS1 bridge for the Optitrack System
- Jetbot running a motor driver to control the wheels + 4 Optitrack markers installed
- DJI Tello running a Tello Driver to be able to fly the drone, with the camera facing downwards + 4 Optitrack markers installed
- A obstacle course with ArUco Markers (101, 102, 0 and 3 for bottom left corner, top right corner, goal and obstacles respectively)

## Step 1
Before being able to run the code, make sure that the Optitrack system is published correctly on ROS2 topics. <br>
See [ros_bridge_instructions.txt](ros_bridge_instructions.txt)

## Step 2
ssh into the Jetbot, make sure that there is a script / driver / code available on the Jetbot that subscribes to /cmd_vel and can move the robot.
Run that code.

## Step 3
Run [path_tracker.py](src/path_tracker.py)

## Step 4
Place the drone in the middle of the obstacle course and run the Tello Driver and launch tello_mapping.py (link here)

## Step 5
If everything ran accordingly, you should see the drone fly up to about 2m, hover, take a picture and publishing it to /tello_map_image.
Then the Jetbot should move (it might need a tiny push at the beginning due to the weakness of the motors) according to the path that has been calculated. You can find the plotted path saved in [Path.png](images/Path.png)




