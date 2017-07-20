import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates

x=0
y=0
phi=0

# Goal
rx=5
ry=5
rphi=-np.pi/2

# Distance between wheel axles
H=0.34

# Design parameters
v=0.5
#phiGain=0.1 # should be <1 ?

# x'=-v*sin(phi)
# y'=v*cos(phi)
# phi'=(v/H)*tan(theta)
def getPos(data):
    offset=H/2
    global x,y,phi
    
    q0 = data.pose[1].orientation.w
    q1 = data.pose[1].orientation.x
    q2 = data.pose[1].orientation.y
    q3 = data.pose[1].orientation.z
    phi = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
    
    x=data.pose[1].position.x+offset*np.sin(phi)
    y=data.pose[1].position.y-offset*np.cos(phi)
    
    
rospy.init_node('lyap')
rat=rospy.Rate(10)

rospy.Subscriber('gazebo/model_states',ModelStates,getPos)
pub=rospy.Publisher('ackermann_cmd',AckermannDrive,queue_size=10)
msg=AckermannDrive()
msg.acceleration=5
msg.steering_angle_velocity=5
msg.speed = v
while (not rospy.is_shutdown()) and ((x-rx)**2+(y-ry)**2>0.1**2):
    
    theta = np.arctan((H/(phi-rphi))*(-np.sin(phi)*(x-rx)+np.cos(phi)*(y-ry)-(x-rx)**2-(y-ry)**2-(phi-rphi)**2))
    msg.steering_angle = np.clip(theta,-np.pi/3,np.pi/3)
    print phi-rphi
    pub.publish(msg)
    rat.sleep()
msg.speed=0
msg.steering_angle=0
pub.publish(msg)

