#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Transform 
import RPi.GPIO as GPIO

def setbit(v, index, x):
	mask = 1 << index
	v &= ~mask
	if x:
		v |= mask;
	return v

def main():
    pub = rospy.Publisher('autoforce_ir', Byte, queue_size=10)
    rospy.init_node('autoforce_ir', anonymous=True)
    rate = rospy.Rate(10)
    
    output_ir = UInt8()
    output_vel = Transform()
    
    sensor_gpio = [1,2,3,4,5,6,7,8]
    GPIO.setmode(GPIO.BOARD)
    for i in sensor_gpio:
        GPIO.setup(i, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
        
    while not rospy.is_shutdown():
		cnt = 0
        for i in sensor_gpio:
        	output[cnt] = GPIO.input(sensor[i])
			setbit(output_ir, cnt, 1)
		else:
			setbit(outpu_ir, cnt, 0)
       			
		rospy.loginfo(output)
       	pub.publish(output)
        rate.sleep()
        
if __name__ == '__main__':
    try:
      	main()
    except rospy.ROSInterruptException:
        pass
        
        

