#!/usr/bin/env python
import message_filters
import rospy
import time
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import math as m
import sys

I = 0.0
errorPasado = 0.0
tiempoPasado = 0.0
anguloPasado = 0.0
stopDec = 0

def pidController(errorActual):
	global I
	global errorPasado
	global tiempoPasado

	vel = rospy.Publisher('/cmd_vel',Twist,queue_size=10)
	move = Twist()

	linear = 0.168-abs(errorActual)*0.12
	tiempoActual = rospy.get_time()
	deltaT = tiempoActual - tiempoPasado
	P = 0.483*errorActual
	I += 0.000000000002286*errorActual*deltaT
	D = 0.663*(errorActual-errorPasado)/deltaT
	angular = P+I+D
	if angular > 1.8:
		angular = 1.8
	if angular < -1.8:
		angular = -1.8
	if stopDec > 0:
		move.linear.x = linear*0.6
		move.angular.z = angular*0.6
	else:
		move.linear.x = linear
		move.angular.z = angular
	tiempoPasado = tiempoActual
	errorPasado = errorActual
	vel.publish(move)
	rospy.Rate(10).sleep()
	return angular, linear

def stop(img):
	global stopDec
	height, width = img.shape[:2]
	img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	signoStop = img[0:height/2,0:width/2]
	height, width = signoStop.shape[:2]
	lower = np.array([0,0,100],dtype="uint8") 
	upper = np.array([40,40,255],dtype="uint8")
	maskFiltro = cv2.inRange(signoStop,lower,upper)
	maskImg = cv2.bitwise_and(signoStop,signoStop,mask=maskFiltro)
	gray = cv2.cvtColor(maskImg, cv2.COLOR_RGB2GRAY)
    	canny = cv2.Canny(gray, 50, 100, apertureSize=3)
	lines = cv2.HoughLinesP(canny,1,np.pi/180,50,minLineLength=60,maxLineGap=10)
	if lines is not None:
		count = 0
		for line in lines:
			x1,y1,x2,y2 = line[0]
			count += 1
		if count > 0:
			if stopDec == 0:
				stopDec += 30
	else:
		if stopDec > 0:
			cv2.putText(maskImg,"Stop detectado, velocidad al 60%",(24,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
			stopDec -= 1
		else:
			stopDec = 0
	return maskImg

	
def lane(img):
	global anguloPasado
	height, width = img.shape[:2]
	img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
	carriles = img[285:height,0:width]
	height, width = carriles.shape[:2]
	lower = np.array([0,100,100],dtype="uint8") 
	upper = np.array([40,255,255],dtype="uint8")
	maskFilter = cv2.inRange(carriles,lower,upper)
	maskImg = cv2.bitwise_and(carriles,carriles,mask=maskFilter)
	gray = cv2.cvtColor(maskImg, cv2.COLOR_RGB2GRAY)
    	canny = cv2.Canny(gray, 50, 150, apertureSize=3)
    	lines = cv2.HoughLinesP(canny,1,np.pi/180,50,minLineLength=60,maxLineGap=10)
	if lines is not None:
    		cd = 0
    		ci = 0
		angd = 0
		angi = 0
		for line in lines:
			x1,y1,x2,y2 = line[0]
			ang = m.atan2(y2-y1,x2-x1)
			if y2 > y1:
				# Lineas de la derecha
				cv2.line(maskImg, (width/2-x2+x1,height-y2+y1), (width/2,height), (0,0,255), 1)
				angd += ang
				cd += 1
				cv2.line(maskImg, (x1,y1), (x2,y2), (0,0,255), 1)
				
			else:
				# Lineas de la izquierda
				cv2.line(maskImg, (width/2,height), (width/2-x1+x2,height-y1+y2), (0,255,0), 1)
				angi += ang
				ci += 1
				cv2.line(maskImg, (x1,y1), (x2,y2), (0,255,0), 1)
		if cd != 0:
			angd /= cd
		if ci != 0:
			angi /= ci
		error = angd + angi + 0.096*anguloPasado
		cv2.line(maskImg, (width/2,height), (width/2+int(100*m.cos(1.570796+error)),int(100*m.sin(1.570796+error))-50), (0,255,255), 1)
		cv2.line(maskImg, (width/2,height), (width/2,0), (255,255,255), 1)
		angular, linear = pidController(error)
		cv2.putText(maskImg,"Velocidad: Linear:{}, Angular:{}".format(linear,angular),(24,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,255,255),2)
		anguloPasado = error
	return maskImg



def process_image(msg):
	try:	
		if cv2.waitKey(1) == 27: 
			cv2.destroyAllWindows()   
		bridge = CvBridge()
		orig = bridge.imgmsg_to_cv2(msg,"rgb8")
		drawImg = orig
		stopImg = stop(drawImg)
		pista = lane(drawImg)
		cv2.imshow('Pista',pista)
		cv2.imshow('Signo stop',stopImg)
	except Exception as err:
		print err



def start_node():
	rospy.init_node('autonomous_driving')
	rospy.loginfo('Iniciando nodo')
	rospy.Subscriber('/camera/rgb/image_raw',Image,process_image)
	rospy.spin()



if __name__ == '__main__':
	try:
		start_node()
	except rospy.ROSInterruptException:
		pass
