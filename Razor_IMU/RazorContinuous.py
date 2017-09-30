import serial
import time
from matplotlib import pyplot as plt
import numpy as np

print 'Hold up Razor in start position'
time.sleep(1)
ser=serial.Serial('/dev/razor')
string_list=[]

plt.axis([000000,500000,-40,-30]) #<---YOU MIGHT WANT TO EDIT THE PLOT AXES
h1, = plt.plot([],[])
plt.ion()
plt.show()
maxDat=0
def update(h1,new_dataX,new_dataY):
	global maxDat
	h1.set_xdata(np.append(h1.get_xdata(), new_dataX))
	h1.set_ydata(np.append(h1.get_ydata(), new_dataY))
	plt.draw()
	if np.abs(new_dataY)>np.abs(maxDat):
		maxDat=new_dataY
	print maxDat

while(True):
	ser.flushInput()
	string_list=(ser.read(150))

	timestamp=[]
	accX=[]
	accY=[]
	accZ=[]
	gyrX=[]
	gyrY=[]
	gyrZ=[]
	magX=[]
	magY=[]
	magZ=[]

	values_string=string_list
	values_list=values_string.split(', ')
	try:
		values_float=map(float,values_list[0:10])
	except:
		continue
	if values_float[0]<5000:
		print 'skipped one'
		continue
	timestamp=(values_float[0])
	print timestamp
	accX=(values_float[1])
	accY=(values_float[2])
	accZ=(values_float[3])
	gyrX=(values_float[4])
	gyrY=(values_float[5])
	gyrZ=(values_float[6])
	magX=(values_float[7])
	magY=(values_float[8])
	magZ=(values_float[9])
	update(h1,timestamp,accZ) ##<--- CHOOSE WHICH PARAMETER TO CALIBRATE HERE

