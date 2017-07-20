# F1-10-simulation


INSTALLATION (With ROS catkin_make):
1. Download src folder
2. Create a new folder
3. Paste the src folder into the new one
4. Run 'catkin_make' in a terminal window from within the folder

HOW TO RUN:
1. Source the ROS package (source ~/'foldername'/devel/setup.bash)
2. Then run 'roslaunch ackermann_vehicle_gazebo ackerman_vehicle.launch' and the Gazebo Simulator should start with the vehicle.

HOW TO DRIVE:
* Source the ROS package and navigate to the controller folder inside src folder. From there, run control_opt.py or control_opt_discrete.py depending on the controller youd like to run

OR
* The vehicle subscribes to the topic ackermann_cmd with the message type AckermannDrive. Use this to create your own python program to control the vehicle.
