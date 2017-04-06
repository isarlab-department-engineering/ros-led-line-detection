#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit
import Adafruit_ADS1x15
from std_msgs.msg import String,Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats
import numpy

def callback(data):
  #  rospy.loginfo(rospy.get_caller_id() +
  #                 " Movement direction: %s", data.data)
  rospy.loginfo(rospy.get_caller_id() + 
		" Detected values: %s", data.data)

  values = data.data

    #setup motor object and its address
  mh = Adafruit_MotorHAT(addr=0x60)

    #at exit code, to auto-disable motor on shutdown
  def turnOffMotors():
        mh.getMotor(1).run(Adafruit_MotorHAT.RELEASE)
        mh.getMotor(2).run(Adafruit_MotorHAT.RELEASE)
        atexit.register(turnOffMotors)

    #setup 2 motors
  m1 = mh.getMotor(1)
  m2 = mh.getMotor(2)

    #setup motors' speed
#  speed = 150
  speed = 150
  motorBalance=14
#  motorBalance = 16 #sperimental value to balance motors' spin
  m1.setSpeed(speed + motorBalance) #left motor
  m2.setSpeed(speed) #right motor

    #read input and set motors

    #rettilineo
  if(values[3]<1200 or values[4]<1200):
        m1.run(Adafruit_MotorHAT.FORWARD)
	m2.run(Adafruit_MotorHAT.FORWARD)
    
    #turn right curva leggera
  elif(values[2]<1200):
       m1.run(Adafruit_MotorHAT.FORWARD)
       m2.run(Adafruit_MotorHAT.FORWARD)
       m1.setSpeed(speed+20)

  # turn right curva secca
  elif (values[1]<1200):
      m1.run(Adafruit_MotorHAT.FORWARD)
      m2.run(Adafruit_MotorHAT.FORWARD)
      m2.setSpeed(speed-110)	
    
  #turn right curva a spigolo
  elif (values[0]<1500):
        m1.run(Adafruit_MotorHAT.FORWARD)
        m2.run(Adafruit_MotorHAT.FORWARD)
        m2.setSpeed(speed-140)


   #turn left leggera
  elif(values[5]<1200):
        m1.run(Adafruit_MotorHAT.FORWARD)
        m2.run(Adafruit_MotorHAT.FORWARD)
       	m2.setSpeed(speed+20)

   #turn left secca
  elif(values[6]<1200):
 	m1.run(Adafruit_MotorHAT.FORWARD)
        m2.run(Adafruit_MotorHAT.FORWARD)
        m1.setSpeed(speed-110)

  #turn left curva a spigolo
  elif(values[7]<1500):
        m1.run(Adafruit_MotorHAT.FORWARD)
        m2.run(Adafruit_MotorHAT.FORWARD)
        m1.setSpeed(speed-140)


    #stop
 # if(values[3]>1300 and values[4]>1300):
  else:
      m1.run(Adafruit_MotorHAT.RELEASE)
      m2.run(Adafruit_MotorHAT.RELEASE)
    

def listener():

    rospy.init_node('listener_ads', anonymous=True)

    rospy.Subscriber("line_follower_topic", numpy_msg(Floats), callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
