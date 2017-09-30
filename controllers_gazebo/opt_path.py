import numpy as np

def get_path(currX, currY, currPhi, goalX, goalY, goalPhi, min_radius=0.32/np.tan(np.pi*20/180), iteration=0):
	# Find the two circular paths for optimal control
	left_curr_X=currX-np.cos(currPhi)*min_radius
	left_curr_Y=currY-np.sin(currPhi)*min_radius
	
	right_curr_X=currX+np.cos(currPhi)*min_radius
	right_curr_Y=currY+np.sin(currPhi)*min_radius
	
	left_goal_X=goalX-np.cos(goalPhi)*min_radius
	left_goal_Y=goalY-np.sin(goalPhi)*min_radius
	
	right_goal_X=goalX+np.cos(goalPhi)*min_radius
	right_goal_Y=goalY+np.sin(goalPhi)*min_radius
	
	cost = np.zeros(4)
	cost[0]= (left_curr_X-left_goal_X)**2+(left_curr_Y-left_goal_Y)**2
	cost[1]= (left_curr_X-right_goal_X)**2+(left_curr_Y-right_goal_Y)**2	
	cost[2]= (right_curr_X-left_goal_X)**2+(right_curr_Y-left_goal_Y)**2
	cost[3]= (right_curr_X-right_goal_X)**2+(right_curr_Y-right_goal_Y)**2
	
#	cost_sorted = np.zeros(4)
#	cost_sorted[0]= (left_curr_X-left_goal_X)**2+(left_curr_Y-left_goal_Y)**2
#	cost_sorted[1]= (left_curr_X-right_goal_X)**2+(left_curr_Y-right_goal_Y)**2	
#	cost_sorted[2]= (right_curr_X-left_goal_X)**2+(right_curr_Y-left_goal_Y)**2
#	cost_sorted[3]= (right_curr_X-right_goal_X)**2+(right_curr_Y-right_goal_Y)**2
#	cost_sorted.sort()
	
	closest=cost.tolist().index(cost.min())
	#print cost
	#print closest
	# Find common tangents through same points in the circles with respect to angular orientation
	if closest is 0:
		sign_curr=1
		sign_goal=1
		curr_x=left_curr_X
		curr_y=left_curr_Y
		goal_x=left_goal_X
		goal_y=left_goal_Y
	elif closest is 1:
		sign_curr=1
		sign_goal=-1
		curr_x=left_curr_X
		curr_y=left_curr_Y
		goal_x=right_goal_X
		goal_y=right_goal_Y
	elif closest is 2:
		sign_curr=-1
		sign_goal=1
		curr_x=right_curr_X
		curr_y=right_curr_Y
		goal_x=left_goal_X
		goal_y=left_goal_Y
	elif closest is 3:
		sign_curr=-1
		sign_goal=-1
		curr_x=right_curr_X
		curr_y=right_curr_Y
		goal_x=right_goal_X
		goal_y=right_goal_Y
		
	if sign_curr*sign_goal>0:
		xx0_1=curr_x - curr_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) + goal_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		xx0_2= curr_x + curr_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) - goal_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		yy0_1= curr_y + curr_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) - goal_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		yy0_2= curr_y - curr_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) + goal_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		xx1_1= goal_x - curr_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) + goal_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		xx1_2= goal_x + curr_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) - goal_y*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		yy1_1= goal_y + curr_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) - goal_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		yy1_2= goal_y - curr_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5) + goal_x*min_radius*(1/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2))**(0.5)
		
		done = False
		for i in range(0,3600):
			if not done:
				x=curr_x+min_radius*np.cos(currPhi + np.pi*(1-sign_curr)/2 + sign_curr*i*np.pi/1800)
				#print x
				y=curr_y+min_radius*np.sin(currPhi + np.pi*(1-sign_curr)/2 + sign_curr*i*np.pi/1800)
				#print x,y
				#print xx0_1,yy0_1,xx1_1,yy1_1
				#print xx0_2,yy0_2,xx1_2,yy1_2
				if (x-xx0_1)**2+(y-yy0_1)**2<0.01**2:
				#	print 'first'
					done = True
					xx0r=xx0_1
					xx1r=xx1_1
					yy0r=yy0_1
					yy1r=yy1_1
					break
				elif (x-xx0_2)**2+(y-yy0_2)**2<0.01**2:
					done = True
					xx0r=xx0_2 
					xx1r=xx1_2
					yy0r=yy0_2
					yy1r=yy1_2
				#	print 'second',xx0r,yy0r,xx1r,yy1r
					break
					#if np.sign((xx1_2-goal_x)*(1)+(yy1_2-goal_y)*((yy1_2-yy0_2)/(xx1_2-xx0_2))) is not sign_goal:
					#	xx1=xx1_1
					#	yy1=yy1_1
		#print xx0r,yy0r,xx1r,yy1r
			
	else:
		curr_x+=0.000000000000
		curr_y+=0.000000000000
		goal_x+=0.000000000000
		goal_y+=0.000000000000
		#print 'else'
		xx0_1= (curr_x*curr_y**2 + curr_x*goal_x**2 - 2*curr_x**2*goal_x + curr_x*goal_y**2 - 2*curr_x*min_radius**2 + 2*goal_x*min_radius**2 + curr_x**3 + curr_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - goal_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_x*curr_y*goal_y)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		xx0_2= (curr_x*curr_y**2 + curr_x*goal_x**2 - 2*curr_x**2*goal_x + curr_x*goal_y**2 - 2*curr_x*min_radius**2 + 2*goal_x*min_radius**2 + curr_x**3 - curr_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) + goal_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_x*curr_y*goal_y)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		yy0_1= (curr_x**2*curr_y + curr_y*goal_x**2 + curr_y*goal_y**2 - 2*curr_y**2*goal_y - 2*curr_y*min_radius**2 + 2*goal_y*min_radius**2 + curr_y**3 - curr_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) + goal_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_x*curr_y*goal_x)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		yy0_2= (curr_x**2*curr_y + curr_y*goal_x**2 + curr_y*goal_y**2 - 2*curr_y**2*goal_y - 2*curr_y*min_radius**2 + 2*goal_y*min_radius**2 + curr_y**3 + curr_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - goal_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_x*curr_y*goal_x)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		xx1_1= (curr_x**2*goal_x - 2*curr_x*goal_x**2 + curr_y**2*goal_x + goal_x*goal_y**2 + 2*curr_x*min_radius**2 - 2*goal_x*min_radius**2 + goal_x**3 - curr_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) + goal_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_y*goal_x*goal_y)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		xx1_2= (curr_x**2*goal_x - 2*curr_x*goal_x**2 + curr_y**2*goal_x + goal_x*goal_y**2 + 2*curr_x*min_radius**2 - 2*goal_x*min_radius**2 + goal_x**3 + curr_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - goal_y*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_y*goal_x*goal_y)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		yy1_1= (curr_x**2*goal_y - 2*curr_y*goal_y**2 + curr_y**2*goal_y + goal_x**2*goal_y + 2*curr_y*min_radius**2 - 2*goal_y*min_radius**2 + goal_y**3 + curr_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - goal_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_x*goal_x*goal_y)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		yy1_2= (curr_x**2*goal_y - 2*curr_y*goal_y**2 + curr_y**2*goal_y + goal_x**2*goal_y + 2*curr_y*min_radius**2 - 2*goal_y*min_radius**2 + goal_y**3 - curr_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) + goal_x*min_radius*(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2 - 4*min_radius**2)**(0.5) - 2*curr_x*goal_x*goal_y)/(curr_x**2 - 2*curr_x*goal_x + curr_y**2 - 2*curr_y*goal_y + goal_x**2 + goal_y**2)
		
		
		done = False
		for i in range(0,3600):
			if not done:
				x=curr_x+min_radius*np.cos(currPhi + np.pi*(1-sign_curr)/2 + sign_curr*i*np.pi/1800)
				#print x
				y=curr_y+min_radius*np.sin(currPhi + np.pi*(1-sign_curr)/2 + sign_curr*i*np.pi/1800)
				#print y
				if (x-xx0_1)**2+(y-yy0_1)**2<0.01**2:
					done = True
					xx0r=xx0_1
					xx1r=xx1_1
					yy0r=yy0_1
					yy1r=yy1_1

				elif (x-xx0_2)**2+(y-yy0_2)**2<0.01**2:
					done = True
					xx0r=xx0_2
					xx1r=xx1_2
					yy0r=yy0_2
					yy1r=yy1_2

	#print xx0_1,xx0_2		
	#print yy0_1,yy0_2		
	#print xx1_1,xx1_2		
	#print yy1_1,yy1_2		
	
	return sign_curr, sign_goal, xx0r, yy0r, xx1r, yy1r, curr_x, curr_y, goal_x, goal_y
