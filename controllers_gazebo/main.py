import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
import opt_path
import time

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
xlog=[]
ylog=[]
tlog = []
t=time.time()
ulog=[]
utlog=[]
t_old1=t
t_old2=t

# Distance between wheel axles
H=0.32

def logU(data):
	global ulog, utlog,t,t_old2
	if time.time()-t_old2>0.005:
		ulog.append(data.steering_angle)
		utlog.append(time.time()-t)
		t_old2=time.time()

def getPos(data):
	offset=-H/2
	global x,y,phi,x_circ,y_circ,t,xlog,ylog,tlog,t_old1
	if time.time()-t_old1>0.005:
		q0 = data.pose[1].orientation.w
		q1 = data.pose[1].orientation.x
		q2 = data.pose[1].orientation.y
		q3 = data.pose[1].orientation.z
		phi = np.arctan2(2*(q0*q3+q1*q2),1-2*(q2*q2+q3*q3))-np.pi/2
		
		x=data.pose[1].position.x-offset*np.sin(phi)
		y=data.pose[1].position.y+offset*np.cos(phi)
		x_circ=data.pose[1].position.x+offset*np.sin(phi)
		y_circ=data.pose[1].position.y-offset*np.cos(phi)
		xlog.append(x)
		ylog.append(y)
		tlog.append(time.time()-t)
		t_old1=time.time()

# Design parameters
v=0.2
aim_gain=1.5

execfile('methods.py')
execfile('discretize.py')
sys,trans=synth_ex()

state=0

rospy.init_node('ctrl')
rate=400
rat=rospy.Rate(rate)

rospy.Subscriber('gazebo/model_states',ModelStates,getPos)
pub=rospy.Publisher('ackermann_cmd',AckermannDrive,queue_size=10)
rospy.Subscriber('ackermann_cmd',AckermannDrive,logU)
time.sleep(1)
fc = True
ts = time.time()
park=False

goto(-5,0,np.pi/2,stop=True)

while(False):
	
#	goto(0,1,np.pi/2)
#	goto(1.25,2.25,0,stop=True)
	if time.time()-ts>15:
		#print 'stop'
		fc=False
	if time.time()-ts>35:
		fc=True
	if time.time()-ts>45:
		park=True
	varies=dict({'park':park,'frontclear':fc})
	var=trans.reaction(state,varies)
	state=int(var[0])
	x_new=float(var[1].get('loc').split('space')[0].split('equal')[1].replace('mmm','-'))
	y_new=float(var[1].get('loc').split('space')[1].split('equal')[1].replace('mmm','-'))
	phi_new=float(var[1].get('loc').split('space')[2].split('equal')[1].replace('mmm','-'))*np.pi/4
	if (x-x_new)**2+(y-y_new)**2<0.2**2:
		continue
	print ''
	print x,y,phi+np.pi/2
	print x_new,y_new,phi_new
	print ''
	goto(x_new,y_new,phi_new,stop=not (fc) or var[1].get('pSpace'))
