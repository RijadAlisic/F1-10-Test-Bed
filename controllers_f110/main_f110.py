import numpy as np
import rospy
import opt_path
import time
from race.msg import drive_param
from geometry_msgs.msg import PoseStamped

x=None
y=None
phi=None
x_circ=None
y_circ=None
first_curve=None
sign_curr=None
sign_goal=None
curr_x=None
curr_y=None
goal_x=None
goal_y=None
xhist=[]
yhist=[]
# Distance between wheel axles
H=0.33

def getPos(data):
	offset=-H/2
	global x,y,phi,x_circ,y_circ,xhist,yhist
	
	q0 = data.pose.orientation.w
	q1 = data.pose.orientation.x
	q2 = data.pose.orientation.y
	q3 = data.pose.orientation.z
	phi = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
	
	x=data.pose.position.x-offset*np.sin(phi)
	y=data.pose.position.y+offset*np.cos(phi)
	x_circ=data.pose.position.x+offset*np.sin(phi)
	y_circ=data.pose.position.y-offset*np.cos(phi)
	xhist+=[x]
	yhist+=[y]
# Design parameters
v=16
aim_gain=1.1

execfile('methods_f110.py')

rospy.init_node('ctrl')
rate=200
rat=rospy.Rate(rate)

rospy.Subscriber('slam_out_pose',PoseStamped,getPos)
pub = rospy.Publisher('drive_parameters', drive_param, queue_size=10)
time.sleep(1)

goto(2,-1,0)
np.save('xvalues.txt',xhist)
np.save('yvalues.txt',yhist)
goto(4,0,0)
np.save('xvalues2.txt',xhist)
np.save('yvalues2.txt',yhist)
