#!/usr/bin/env python
import rospy
from std_msgs.msg import UInt8
from geometry_msgs.msg import Transform
import RPi.GPIO as GPIO

def setbit(v, index, x):
    mask = 1 << index
    v &= ~mask
    if x:
        v |= mask
    return v

def main():

    sensor_gpio = [6,13,19,26,5,5,5,5]
    raw = [0,0,0,0,0,0,0,0]
    output_ir = UInt8();
    output_vel = Transform();
    scale_val = -1;

    raw_pub = rospy.Publisher('autoforce/ir_raw', UInt8, queue_size=10)
    mov_pub = rospy.Publisher('autoforce/ir_mov', Transform, queue_size=10)

    rospy.init_node('autoforce_ir_node', anonymous=True)
    rate = rospy.Rate(10)

    GPIO.setmode(GPIO.BOARD)

    for i in sensor_gpio:
        GPIO.setup(i, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

    while not rospy.is_shutdown():
        cnt = 0
        for i in sensor_gpio:
            if(GPIO.input(sensor[i])):
                setbit(output_ir, cnt, 1)
                raw[cnt] = 1;
            else:
                setbit(output_ir, cnt, 0)
                raw[cnt] = 0;
            cnt += 1

        raw_pub.publish(output)

        x = raw[0] + (0.707 * raw[1]) - (0.707 * raw[3]) - raw[4] - (0.707 * raw[5]) + (0.707 * raw[7])
        y = (0.707 * raw[1]) + raw[2] + (0.707 * raw[3]) - (0.707 * raw[5]) - raw[6] - (0.707 * raw[7])

        output_vel.translation.x = scale_val * x
        output_vel.translation.y = scale_val * y

        mov_pub.publish(output_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

