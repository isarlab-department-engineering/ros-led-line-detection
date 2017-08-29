#!/usr/bin/env python

import Adafruit_ADS1x15
import time
import rospy
import numpy
from std_msgs.msg import String,Int32MultiArray
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

def talker():
    pub = rospy.Publisher('line_follower_topic', numpy_msg(Floats),queue_size=10)
    rospy.init_node('talker_ads', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    adc_dx = Adafruit_ADS1x15.ADS1015(address=0x48, busnum=1)
    adc_sx = Adafruit_ADS1x15.ADS1015(address=0x49, busnum=1)

    #msg= Int32MultiArray
    while not rospy.is_shutdown():
	values = numpy.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=numpy.float32)
	#msg = [0]*8
	for i in range(4):
		values[i] = adc_dx.read_adc(i, gain=1)
                values[i+4] = adc_sx.read_adc(i, gain=1)

   # while not rospy.is_shutdown():
	 # Read all the ADC channel values in a list.
	#values = [0]*8
    	#for i in range(4):
        # Read the specified ADC channel using the previously set gain value.
     	#	 values[i] = adc_dx.read_adc(i, gain=1)
	#	 values[i+4] = adc_sx.read_adc(i, gain=1)

	rospy.loginfo(values)
	pub.publish(values)
	rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
