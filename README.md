# F1-10-Test-Bed

README FOR F1TENTH BUILD, August 2017:

Most of the tutorial found on the F1tenth website is still valid, http://f1tenth.org/car-assembly. This readme will focus on the things that did not work, because of outdated software or non-clear diagrams. In addition to this, a Gazebo-simulator of the F1tenth car is also shown here.

Teensy wiring diagram:
1. Follow the Build tutorial until step 4. 
2. Connect the ground cables to each other, (Black wires from the Traxxas vehicle, GND from the Teensy).
3. Connect the voltage supply (Red cable) from the EXT connector to SRV (Red cable).
4. Connect the signaling cables (White) to the Teensy. One should go to connection 5 and the other to connection 6.

In order to use the python codes available in this repo, you will want to change the names of the external boards as shown inside the ubuntu system in order to not need to change them on every startup. There are tutorials online that show you how to do this. The names that are used here are '/dev/teensy' for the teensy, '/dev/hokuyo' for the LIDAR and '/dev/razor' for the IMU.

You may now return to the tutorial, starting from the section "Mounting the Teensy board". However, you may want to do the next step before mounting the IMU.

Calibrating the IMU:
The code for the AHRS linked to by the F1tenth website is only working for older SEN-10736 models, which are no longer sold. However, the MPU-9250 is a completely redesigned IMU, with better stability and accuracy. The AHRS code that is provided in this package uses the same code as the "old" for calculations, but within a new framework that is comaptible with the current MPU-9250.
CALIBRATION:
1. Accelerometer; Hold your IMU (preferably tape it to a book or something similar that you can flip around). Open RazorContinuous.py and set the input to the finction in the last line to 'AccX' and save the file. Now orient your IMU so that the X-axis is parallel to the gravity vector (i.e, point it first straigt down). Now run the RazorContinuous script. It will start to show the maximum absolute value obtained by the IMU in the current session. Rotate the IMU SLOWLY(!) in order to find the largest absolute value, and then enter it into the code section CALIBRATION. Do the same for the opposite x-axis (point it up). Redo everything for Y and Z directions.
2. Magnetometer; Do exactly the same as in the previous step, however, instead of aligned the axes with the gravity vector, you will now have to align them with the magnetic vector (pointing north and down in the upper half hemisphere).
3. Gyroscope; This one is easier, as you will only need to find the average output of the gyroscope while it is stationary. Place the IMU on a surface (such as the table) and find the average output generated in the three different axes. Use the plot provided in the RazorContinuous.py file (you may have to edit the x and y bounds).

INSTALLATION: In order to install it, simply plug your MPU-9250 to the computer, and upload the code to the IMU using the Arduino IDE.

Finish the building tutorial. And do the drive tutorial.

When you are done with the SLAM tutorial, do the following in order to get the correct launch file 
1. Open a terminal and navigate to the F1tenth code folder, week 3.
2. Write "sudo cp tutorial.launch /opt/ros/indigo/share/hector_slam_launch/launch/f110.launch"

This should connect the SLAM nodes correctly.


Installation of the F110 on the Gazebo simulator

NECESSARY ROS PACKAGES (That are not included in ros-indigo-desktop-full)
* ros-indigo-joint-state-controller
* ros-indigo-effort-controller
* ros-indigo-ackermann-msgs

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
