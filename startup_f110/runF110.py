from subprocess import Popen as Popen
import time

Popen('roscore',shell=True)
time.sleep(2)
Popen('python IMU.py',shell=True)
Popen('rosrun rosserial_python serial_node.py /dev/teensy',shell=True)
Popen('rosrun hokuyo_node hokuyo_node _port:=/dev/hokuyo',shell=True)
time.sleep(20)
Popen('roslaunch hector_slam_launch f110.launch', shell=True)
Popen('python talker.py',shell=True)


