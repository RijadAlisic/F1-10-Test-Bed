import serial
import time
from matplotlib import pyplot as plt
import numpy as np
import rospy
from geometry_msgs.msg import Twist,Vector3
from std_msgs.msg import UInt32
from nav_msgs.msg import Odometry
import tf

print 'Warming up the IMU...'
rospy.init_node('imu_node')
rate=rospy.Rate(100)

br=tf.TransformBroadcaster()
pubTime=rospy.Publisher('/imu/timestamp',UInt32,queue_size=10)
pubAccAng=rospy.Publisher('/imu/AccAndAng',Twist,queue_size=10)
pubMag=rospy.Publisher('/imu/mag',Vector3,queue_size=10)
pubOdom=rospy.Publisher('/odom',Odometry,queue_size=10)
ser=serial.Serial('/dev/razor')
time.sleep(30) #Time for the IMU to warm up
print 'Done! Starting to publish...'
msgT=UInt32()
msgAA=Twist()
msgM=Vector3()
msgO=Odometry()
x=0#0.2475
y=0#-0.057
msgO.header.frame_id='odom'
msgO.child_frame_id='base_frame'

scale=1

fixScale=True

while(True):
	ser.flushInput()
	string_list=(ser.read(150))

	values_string=string_list
	values_list=values_string.split(', ')
	try:
		values_float=map(float,values_list[0:13])
	except:
		continue
	if values_float[0]<3000:
	#	print 'skipped one'
		continue
	while(fixScale and np.abs(values_float[3])/scale>15):
		scale=scale*2
	fixScale=False

	msgT.data=(values_float[0])
	pubTime.publish(msgT)
	#print timestamp
	
	s1=np.sin((values_float[10]))
	s2=np.sin((values_float[11]))
	s3=np.sin((values_float[12]))
	c1=np.cos((values_float[10]))
	c2=np.cos((values_float[11]))
	c3=np.cos((values_float[12]))
	
	R=np.matrix([[c2*c3,-c2*s3,s2],[c1*s3+c3*s1*s2,c1*c3-s1*s2*s3,-c2*s1],[s1*s3-c1*c3*s2,c3*s1+c1*s2*s3,c1*c2]])
	
	g=np.matrix([[0],[0],[-9.796]])
	grot=R*g
	msgAA.linear.x=(values_float[1])/scale+grot[0]
	msgAA.linear.y=(values_float[2])/scale-grot[1]
	msgAA.linear.z=(values_float[3])/scale+grot[2]
	msgAA.angular.x=(values_float[4])
	msgAA.angular.y=(values_float[5])
	msgAA.angular.z=(values_float[6])
	pubAccAng.publish(msgAA)
	msgM.x=(values_float[7])
	msgM.y=(values_float[8])
	msgM.z=(values_float[9])
	pubMag.publish(msgM)
	x=x+msgO.twist.twist.linear.x/100+msgAA.linear.x/100**2/2
	msgO.pose.pose.position.x=x#-0.2475*np.cos(msgO.pose.pose.orientation.z)-0.057*np.sin(msgO.pose.pose.orientation.z)
	y=y+msgO.twist.twist.linear.y/100+msgAA.linear.y/100**2/2
	msgO.pose.pose.position.y=y#-0.2475*np.sin(msgO.pose.pose.orientation.z)+0.057*np.cos(msgO.pose.pose.orientation.z)
	msgO.pose.pose.position.z=msgO.pose.pose.position.z+msgO.twist.twist.linear.z/100+msgAA.linear.z/100**2/2
	msgO.pose.pose.orientation.x=np.mod((values_float[10])+2*np.pi,2*np.pi)-np.pi
	msgO.pose.pose.orientation.y=(values_float[11])
	msgO.pose.pose.orientation.z=(values_float[12])

	msgO.twist.twist.linear.x=msgO.twist.twist.linear.x+msgAA.linear.x/100
	msgO.twist.twist.linear.y=msgO.twist.twist.linear.y+msgAA.linear.y/100
	msgO.twist.twist.linear.z=msgO.twist.twist.linear.z+msgAA.linear.z/100
	msgO.twist.twist.angular.x=msgAA.angular.x
	msgO.twist.twist.angular.y=msgAA.angular.y
	msgO.twist.twist.angular.z=msgAA.angular.z
	pubOdom.publish(msgO)
	br.sendTransform((msgO.pose.pose.position.x, msgO.pose.pose.position.y, 0), tf.transformations.quaternion_from_euler(0, 0, msgO.pose.pose.orientation.z),rospy.Time.now(),'base_frame','odom')
	rate.sleep()
	
	
	
