import numpy as np
import rospy
import opt_path
import time
from race.msg import drive_param


def findR(x,y,phi,aim_gain,msg,theta,v,H,rate,sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y,final_curve,first_curve, line):
	global x_circ,y_circ
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
		side=-np.sign((r_temp_x-x)*((yy1-yy0)/(xx1-xx0))+(r_temp_y-y)*(-1))
		first_curve=False
		line=True
	else:
		x1=curr_x
		y1=curr_y
		r1=0.32/np.tan(np.pi/9)
		
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
		r1=0.32/np.tan(np.pi/9)
		
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
			r_temp_x=xc
			r_temp_y=yc
			r_temp_phi=sign_curr*np.arctan((xc-curr_x)/(yc-curr_y))
			orient=sign_curr
			first_curve=True
		else:

			r_temp_x=xg
			r_temp_y=yg
			r_temp_phi=sign_goal*np.arctan((xg-goal_x)/(yg-goal_y))
			line=False
			final_curve=True
			orient=sign_goal
		
		side=orient*np.sign(np.sqrt((r_temp_x-x)**2+(r_temp_y-y)**2)-min_radius)
	
	r_temp_phi=r_temp_phi-np.pi/2
	r_temp_phi=np.mod(r_temp_phi+np.pi,2*np.pi)-np.pi
	futurePhi=np.mod(futurePhi+np.pi,2*np.pi)-np.pi
	
	return r_temp_x, r_temp_y, r_temp_phi, side, futureX, futureY, futurePhi, final_curve, first_curve, line	
	

def goto(rxx,ryy,rphi):
	rx=rxx-H*np.cos(rphi)
	ry=ryy-H*np.sin(rphi)
	xhist=[]
	yhist=[]
	global pub,min_radius,x,y,phi,x_circ,y_circ,first_curve,sign_curr,sign_goal,v,curr_x,curr_y,goal_x,goal_y
	rphi=rphi-np.pi/2
	# Generate optimal path
	e_old=0
	time.sleep(3)
	print x,y,phi,rx,ry,rphi
	sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y=opt_path.get_path(x,y,phi,rx,ry,rphi)
	print sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y, rx, ry
	final_curve=False
	first_curve=True
	line=False
	msg=drive_param()
	#msg.acceleration=20
	#msg.angle_velocity=50000
	pub.publish(msg)
	out=sign_curr*np.pi/12
	msg.angle=sign_curr*np.pi/12
	pub.publish(msg)
	msg.velocity = v
	side=0
	min_radius=0.32/np.tan(np.pi/10)
	time.sleep(1)
	pub.publish(msg)
	time.sleep(0.05)
	stt=time.time()
	r_temp_x=x
	r_temp_y=y
	r_temp_phi=phi
	phi_old=phi
	I=0
	u=out
	
	inputs=np.ones([2,1])*sign_curr*np.pi/6*0
	bnd=(-np.pi*25/180,np.pi*25/180)
	bnds=(bnd,bnd)
	#print bnds
	while (not rospy.is_shutdown()) and (np.abs(x-rx)>0.1 or np.abs(y-ry)>0.1):
		r_temp_x_old=r_temp_x
		r_temp_y_old=r_temp_y
		r_temp_phi_old=r_temp_phi
		theta=msg.angle+0.001
		
		r_temp_x, r_temp_y, r_temp_phi, side, futureX, futureY, futurePhi, final_curve, first_curve,line = findR(x,y,phi,aim_gain,msg,theta,v,H,rate,sign_curr, sign_goal, xx0, yy0, xx1, yy1, curr_x, curr_y, goal_x, goal_y,final_curve,first_curve,line)
		
		m=0.1 # Design parameters
		aa=1
		ff=1
		if first_curve:
			output=-sign_curr*np.pi/180*25*(np.arctan(((x-curr_x)**2+(y-curr_y)**2-min_radius**2)*10)/(np.pi/2))*500#*np.sign((x+H*np.sin(phi)-curr_x)**2+(y-H*np.cos(phi)-curr_y)**2-min_radius**2)
			theta=output
			
		elif final_curve:
			output=-sign_goal*np.pi*25/180*(np.arctan(((x-goal_x)**2+(y-goal_y)**2-min_radius**2)*10)/(np.pi/2))*500#np.sign((x+H*np.sin(phi)-goal_x)**2+(y-H*np.cos(phi)-goal_y)**2-min_radius**2)#v/H*2
			theta=output
			
			
		elif line and np.abs((x-xx0)*(yy1-yy0)-(xx1-xx0)*(y-yy0))<0.005:
			phi0=np.arctan2(yy1-yy0,xx1-xx0)
			phi0=phi0-np.pi/2
			phi0=np.mod(phi0+np.pi,2*np.pi)-np.pi
			f=(x-H*np.sin(phi)-xx0)*(yy1-yy0)/(xx1-xx0)-y-H*np.cos(phi)+yy0
			a=np.mod(phi-phi0+np.pi,2*np.pi)-np.pi
			b=f+m*a
			fp=(-np.sin(phi)*(yy1-yy0)/(xx1-xx0)-np.cos(phi))*v
			if a>0:
				output=(-2*f*fp-ff*f**2-a**2)
			elif a<0:
				output=(2*f*fp+ff*f**2+a**2)
			else:
				output=0
			theta=np.arctan(H/v*output)
		else:
			output=np.sign((x-H*np.sin(phi)-xx0)*(yy1-yy0)-(xx1-xx0)*(y+H*np.cos(phi)-yy0))*v/H
			theta=np.arctan(H/v*output)
		
		msg=drive_param()
		msg.velocity=v
		u=np.clip(theta,-np.pi*25/180,np.pi*25/180)
		print np.round(-ang_to_map(u)),x,y
		msg.angle=np.round(-ang_to_map(u))
		pub.publish(msg)
		rat.sleep()
		stt=time.time()
		#print u
		
	#msg.angle=sign_goal*np.pi*25/180
	#pub.publish(msg)
	#time.sleep(0.8)
	
	while (not rospy.is_shutdown()) and np.sqrt ((x-rxx)**2+(y-ryy)**2)>0.1:#(np.abs(x-rxx)>0.04 or np.abs(y-ryy)>0.04):
		#print 'movedon',rx,rxx,ry,ryy
		if np.abs((x-rx)*(ryy-ry)-(rxx-rx)*(y-ry))<0.005:
			phi0=np.arctan2(ryy-ry,rxx-rx)
			phi0=phi0-np.pi/2
			phi0=np.mod(phi0+np.pi,2*np.pi)-np.pi
			f=(x-H*np.sin(phi)-rx)*(ryy-ry)/(rxx-rx)-y-H*np.cos(phi)+ry
			a=np.mod(phi-phi0+np.pi,2*np.pi)-np.pi
			b=f+m*a
			fp=(-np.sin(phi)*(ryy-ry)/(rxx-rx)-np.cos(phi))*v
			if a>0:
				output=(-2*f*fp-ff*f**2-a**2)
			elif a<0:
				output=(2*f*fp+ff*f**2+a**2)
			else:
				output=0
			theta=np.arctan(H/v*output)
		else:
			output=np.sign((x-H*np.sin(phi)-rx)*(ryy-ry)-(rxx-rx)*(y+H*np.cos(phi)-ry))*v/H
			theta=np.arctan(H/v*output)
		
		msg=drive_param()
		msg.velocity=v
		u=np.clip(theta,-np.pi*25/180,np.pi*25/180)
		print np.round(-ang_to_map(u)),x,y,'last'
		msg.angle=np.round(-ang_to_map(u))
		pub.publish(msg)
		rat.sleep()
		stt=time.time()
	
	print 'finished'
	msg.velocity=0
	msg.angle=0
	pub.publish(msg)
	

def ang_to_map(phi):
    a=-0.000029017470541
    b=0.007903143833765
    c=0.002618600057425-np.abs(phi)
    
    out=np.sign(phi)*(-b+np.sqrt(b**2-4*a*c))/(2*a)
    return out
    
    
def circFunc(inp):
	global x,y,phi,curr_x,curr_y,goal_x,goal_y,min_radius,H,rate,first_curve,sign_curr,sign_goal,v,H
	
	if first_curve:
		xmid=curr_x
		ymid=curr_y
		sign=sign_curr
	else:
		xmid=goal_x
		ymid=goal_y
		sign=sign_goal
	xi=x+H*np.sin(phi)
	yi=y-H*np.cos(phi)
	phii=phi
	score=0
	for i in range(0,100):
		th=inp[0]
		phiold=phii
		phii=phii+v/H*np.tan(th)/10
		xi=xi+H*(np.cos(phii)-np.cos(phiold))/(np.tan(th)+0.0000001)
		yi=yi+H*(np.sin(phii)-np.sin(phiold))/(np.tan(th)+0.0000001)
		
		score=score+np.abs((xi-xmid)**2+(yi-ymid)**2-min_radius**2)+(np.mod(phii-np.arctan2(yi-ymid,xi-xmid)*sign,2*np.pi))**2
	#print 's',score,inp,xi,phii,phiold,H,th
	return score
