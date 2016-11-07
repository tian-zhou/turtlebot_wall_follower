#### README

1. Start Gazebo simulation envrioment by typing
	roslaunch turtlebot_gazebo turtlebot_world.launch

2. The robot is equipped with a hokuyo on top. Facing front direction of the robot. 
 
3. Drag the robot to a good start position using the GUI. Get familar with Gazebo view changes will help you a lot. Like using SHIFT and scroll up-down

4. Launch all the F1-drive codes with 
	roslaunch f1_drive f1_drive.launch
	It will start three problems:
	- car_brain.py
	- lidar_read.py
	- move_car.py 
	For details of each problem, read the (commented) source code

5. Now the car should start following the right wall 


####Things that you should change
1. move_car.py works with Turtlebot, to make it work with ackerman car, you need to change this script, using the driver/interface Prof. provides

2. Currently the robot moves in a constant speed. After knowing the map and selecting way points, you can use that to calculate the drive error and publish that on topic "drive_e", then the car_brain.py program will calculate the drive control and publish on topic "drive_control", which you can use to control the car.

3. You can change PID parameters in car_brain.py

4. Be **very** careful with line 101 and 102 of lidar_read.py. These indexes need to be changed based on the placement of LIDAR on car. You need to know which ray in LIDAR corresponds to the right direction of car (and right ahead direction). You need to change based on different LIDAR placements. In turtlebot, the LIDAR is put upside down?, so the last value corresponds t o the right. Need to be careful! 

5. In lidar_read.py, you can change theta, dis_rwall_T, and step_into_future. They are based on the UPenn video tutorial. Taa should be familiar with the meaning of these metrics.