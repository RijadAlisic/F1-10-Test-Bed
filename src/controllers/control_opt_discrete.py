import numpy as np
import rospy
from ackermann_msgs.msg import AckermannDrive
from gazebo_msgs.msg import ModelStates
import opt_path
import time

x=00000000000
y=000000000000000
phi=-np.pi/2

# Goal
rx=5
ry=-5
rphi=0

# Distance between wheel axles
H=0.34

# Design parameters
v=0.2
aim_gain=1

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
    
def findR(x,y,phi,aim_gain,msg,theta,v,H,rate,sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y,final_curve):
	futurePhi = phi+(aim_gain-1)*(v/(H*rate))*np.tan(theta)
	futureX=x+aim_gain*(H/(np.tan(theta)))*(np.cos((futurePhi)+np.tan(theta)*v/(H*rate))-np.cos((phi)))
	futureY=y+aim_gain*(H/(np.tan(theta)))*(np.sin((futurePhi)+np.tan(theta)*v/(H*rate))-np.sin((phi)))
	futurePhi = phi+(aim_gain)*(v/(H*rate))*np.tan(theta)
	
	xlinear=(futureX-(yy0-futureY-xx0*(yy1-yy0)/(xx1-xx0+0.0000001))*(yy1-yy0)/(xx1-xx0+0.0000001))/(1+(yy1-yy0)**2/(xx1-xx0+0.0000001)**2)
	ylinear=yy0+(xlinear-xx0)*(yy1-yy0)/(xx1-xx0+0.0000001)
	
	if (xlinear<np.max([xx0,xx1])+0.0000001 and xlinear>np.min([xx0,xx1])-0.0000001) and (ylinear<np.max([yy0,yy1])+0.0000001 and ylinear>np.min([yy0,yy1])-0.0000001) and not final_curve:
		r_temp_x=xlinear
		r_temp_y=ylinear
		r_temp_phi=np.arctan((yy1-yy0)/(xx1-xx0+0.0000001))+0.5*(1-np.sign(xx1-xx0))*np.pi
		if r_temp_phi>2*np.pi:
			r_temp_phi=r_temp_phi-2*np.pi
		#y-yy0=(x-xx0)*(yy1-yy0)/(xx1-xx0)
		side=-np.sign((r_temp_x-x)*((yy1-yy0)/(xx1-xx0))+(r_temp_y-y)*(-1))
		
	else:
		
		# (x-x1)**2+(y-y1)**2=r1**2
		# y=y1+-sqrt(r1**2-(x-x1)**2)
		# r=sqrt((x-x0)**2+(y1-y0+-sqrt(r1**2-(x-x1)**2))**2)
		# 0=((x-x0)+(y1-y0+-sqrt(r1**2-(x-x1)**2))*(-+(x-x1)/sqrt(r1**2-(x-x1)**2)))/sqrt
		# 0=(x1-x0)-+(y1-y0)*((x-x1)/sqrt(r1**2-(x-x1)**2))
		# (x1-x0)**2/(y1-y0)**2*(r1**2-(x-x1)**2)=(x-x1)**2
		# x=x1+-r1/sqrt(1+(y1-y0)**2/(x1-x0)**2)
		# y=y1+-r1/sqrt(1+(x1-x0)**2/(y1-y0)**2)
		x1=curr_x
		y1=curr_y
		r1=0.34/np.tan(np.pi/3)
		
		xcp=x1+r1/np.sqrt(1+(y1-futureY)**2/(x1-futureX)**2)
		xcn=x1-r1/np.sqrt(1+(y1-futureY)**2/(x1-futureX)**2)
		ycp=y1+r1/np.sqrt(1+(x1-futureX)**2/(y1-futureY)**2)
		ycn=y1-r1/np.sqrt(1+(x1-futureX)**2/(y1-futureY)**2)
		
		cost=np.zeros(4)
		cost[0]=(xcp-futureX)**2+(ycp-futureY)**2
		cost[1]=(xcp-futureX)**2+(ycn-futureY)**2
		cost[2]=(xcn-futureX)**2+(ycp-futureY)**2
		cost[3]=(xcn-futureX)**2+(ycn-futureY)**2
		
		closest=cost.tolist().index(cost.min())
		
		if closest is 0:
			xc=xcp
			yc=ycp
		elif closest is 1:
			xc=xcp
			yc=ycn
		elif closest is 2:
			xc=xcn
			yc=ycp
		elif closest is 3:
			xc=xcn
			yc=ycn
	
		x1=goal_x
		y1=goal_y
		r1=0.34/np.tan(np.pi/3)
		
		xgp=x1+r1/np.sqrt(1+(y1-futureY)**2/(x1-futureX)**2)
		xgn=x1-r1/np.sqrt(1+(y1-futureY)**2/(x1-futureX)**2)
		ygp=y1+r1/np.sqrt(1+(x1-futureX)**2/(y1-futureY)**2)
		ygn=y1-r1/np.sqrt(1+(x1-futureX)**2/(y1-futureY)**2)
		
		cost=np.zeros(4)
		cost[0]=(xgp-futureX)**2+(ygp-futureY)**2
		cost[1]=(xgp-futureX)**2+(ygn-futureY)**2
		cost[2]=(xgn-futureX)**2+(ygp-futureY)**2
		cost[3]=(xgn-futureX)**2+(ygn-futureY)**2
		closest=cost.tolist().index(cost.min())
		
		if closest is 0:
			xg=xgp
			yg=ygp
		elif closest is 1:
			xg=xgp
			yg=ygn
		elif closest is 2:
			xg=xgn
			yg=ygp
		elif closest is 3:
			xg=xgn
			yg=ygn
	
		cost = np.zeros(2)
		cost[0]=(xc-futureX)**2+(yc-futureY)**2
		cost[1]=(xg-futureX)**2+(yg-futureY)**2
		closest=cost.tolist().index(cost.min())
		
		if closest is 0:
			#print 'first circ'
			r_temp_x=xc
			r_temp_y=yc
			# gradient is (-(y-y0),x-x0)
			r_temp_phi=sign_curr*np.arctan((xc-curr_x)/(yc-curr_y))#+0.5*(1-np.sign(yc-curr_y))*np.pi
			orient=sign_curr
		else:
			#print 'last circ'
			#if not final_curve:
			#	msg.steering_angle=sign_goal*np.pi/6
			#	pub.publish(msg)
			r_temp_x=xg
			r_temp_y=yg
			r_temp_phi=sign_goal*np.arctan((xg-goal_x)/(yg-goal_y))#+0.5*(1-np.sign(yg-goal_y))*np.pi
			final_curve=True
			orient=sign_curr
		
		side=orient*np.sign(np.sqrt((r_temp_x-x)**2+(r_temp_y-y)**2)-min_radius)
	
	r_temp_phi=r_temp_phi-np.pi/2
	r_temp_phi=np.mod(r_temp_phi+np.pi,2*np.pi)-np.pi
	futurePhi=np.mod(futurePhi+np.pi,2*np.pi)-np.pi
	
	return r_temp_x, r_temp_y, r_temp_phi, side, futureX, futureY, futurePhi, final_curve
	#print r_temp_x
	#print r_temp_y
#	while np.abs(futurePhi-r_temp_phi)>np.pi-0.01:
		#print r_temp_phi
		#print futurePhi
#		if futurePhi-r_temp_phi>np.pi+0.01:
#			futurePhi=futurePhi-2*np.pi
#		if futurePhi-r_temp_phi<np.pi-0.01:
#			futurePhi=phi+2*np.pi
#	return r_temp_x, r_temp_y, r_temp_phi, side, futureX, futureY, futurePhi, final_curve
	
    
    
rospy.init_node('ctrl')
rate=100
rat=rospy.Rate(rate)

rospy.Subscriber('gazebo/model_states',ModelStates,getPos)
pub=rospy.Publisher('ackermann_cmd',AckermannDrive,queue_size=10)


# Generate optimal path
time.sleep(3)
print x,y
sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y=opt_path.get_path(x,y,phi,rx,ry,rphi)
print sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y
final_curve=False
#exit()
msg=AckermannDrive()
msg.acceleration=20
msg.steering_angle_velocity=20
pub.publish(msg)
msg.steering_angle=sign_curr*np.pi/6
pub.publish(msg)
msg.speed = v
side=0
min_radius=0.34/np.tan(np.pi/6)
time.sleep(1)
pub.publish(msg)
time.sleep(0.05)
stt=time.time()
r_temp_x=x
r_temp_y=y
r_temp_phi=phi
while (not rospy.is_shutdown()) and ((x-rx)**2+(y-ry)**2>0.10**2):
	r_temp_x_old=r_temp_x
	r_temp_y_old=r_temp_y
	r_temp_phi_old=r_temp_phi
	theta=msg.steering_angle+0.001
	
	r_temp_x, r_temp_y, r_temp_phi, side, futureX, futureY, futurePhi, final_curve = findR(x,y,phi,aim_gain,msg,theta,v,H,rate,sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y,final_curve)
	
	dx=r_temp_x_old-x
	dy=r_temp_y_old-y
	dphi=np.mod(r_temp_phi_old-phi+np.pi,2*np.pi)-np.pi
	dnextX=r_temp_x-futureX
	dnextY=r_temp_y-futureY	
	side_con=side
	
	tan=min_radius*rate/v*(dphi+side*np.sqrt(np.abs(0.5*dx**2+0.5*dy**2+0.5*dphi**2-dnextX**2-dnextY**2)))
	theta=np.arctan(tan)
	
#	while 0.5*dx**2+0.5*dy**2+0.5*dphi**2-dnextX**2-dnextY**2<0:
#		theta=theta+side*0.01
#		print 'theta'
#		print theta
#		print 0.5*dx**2+0.5*dy**2+0.5*dphi**2-dnextX**2-dnextY**2
#		r_temp_x, r_temp_y, r_temp_phi, side, futureX, futureY, futurePhi, final_curve = findR(x,y,phi,aim_gain,msg,theta,v,H,rate,sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y, final_curve)
#		dphi=r_temp_phi_old-phi
#		dnextX=r_temp_x-futureX
#		dnextY=r_temp_y-futureY
#		print dnextX
#		print dnextY
	
	print 'done'
	
	# V=(x-x0)**2+(y-y0)**2+(phi-phi0)**2
	# V'/2=(x-x0)*(-v*sin(phi))+(y-y0)*v*cos(phi)+(phi-phi0)*v/H*tan(theta)
	#xk+1=xk-int vsinphik
	#yk+1=yk+ int vcosphik
	#phik+1=phikv+deltat*v/H*tan(thetak)

	msg.speed=np.clip(v,-0.2,0.2)
	
	if final_curve and ((futureX-rx)**2+(futureY-ry)**2<0.10**2):
		msg.speed=np.clip(np.sqrt((x-rx)**2+(y-ry)**2),0,0.5)
		#aim_gain=aim_gain*msg.speed/v
	msg.steering_angle=np.clip(theta,-np.pi/3,np.pi/3)
	
	#print xx0,xlinear,xx1
	#print yy0,ylinear,yy1
	print x-r_temp_x
	print y-r_temp_y
	print phi-r_temp_phi
	print theta
	print ''
	pub.publish(msg)
	print time.time()-stt
	rat.sleep()
	stt=time.time()
	
print 'finished'
	
msg.speed=0
msg.steering_angle=0
pub.publish(msg)

	
			
