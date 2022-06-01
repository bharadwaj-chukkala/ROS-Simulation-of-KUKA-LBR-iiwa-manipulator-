#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

def manipulator_control():
    rospy.init_node('manipulator_control') 
    motor_1 = rospy.Publisher('/ur10e/motor_1_revolute_controller/command', Float64, queue_size=10)
    motor_2 = rospy.Publisher('/ur10e/motor_2_revolute_controller/command', Float64, queue_size=10)
    motor_3 = rospy.Publisher('/ur10e/motor_3_revolute_controller/command', Float64, queue_size=10)
    motor_4 = rospy.Publisher('/ur10e/motor_4_revolute_controller/command', Float64, queue_size=10)
    motor_5 = rospy.Publisher('/ur10e/motor_5_revolute_controller/command', Float64, queue_size=10)
    motor_6 = rospy.Publisher('/ur10e/motor_6_revolute_controller/command', Float64, queue_size=10)
    rate = rospy.Rate(1) 
    rospy.loginfo("Data is being sent")  
    while not rospy.is_shutdown():
        twist = Float64()

        #Initial pose
        twist.data = 0 * 0.01744
        motor_1.publish(twist)
        twist.data = 0 * 0.01744
        motor_2.publish(twist)
        twist.data = 0 * 0.01744
        motor_3.publish(twist)
        twist.data = 0 * 0.01744
        motor_4.publish(twist)
        twist.data = 0 * 0.01744
        motor_5.publish(twist)
        twist.data = 0* 0.01744
        motor_6.publish(twist)
        rate.sleep()

        #picking pose
        twist.data = 0 * 0.01744
        motor_1.publish(twist)
        twist.data = 0 * 0.01744
        motor_2.publish(twist)
        twist.data = -90 * 0.01744
        motor_3.publish(twist)
        twist.data = 0 * 0.01744
        motor_4.publish(twist)
        twist.data = 90 * 0.01744
        motor_5.publish(twist)
        twist.data = 0* 0.01744
        motor_6.publish(twist)
        rate.sleep()

        #Initial pose
        twist.data = 0 * 0.01744
        motor_1.publish(twist)
        twist.data = 0 * 0.01744
        motor_2.publish(twist)
        twist.data = 0 * 0.01744
        motor_3.publish(twist)
        twist.data = 0 * 0.01744
        motor_4.publish(twist)
        twist.data = 0 * 0.01744
        motor_5.publish(twist)
        twist.data = 0* 0.01744
        motor_6.publish(twist)
        rate.sleep()

        #picking pose
        twist.data = 0 * 0.01744
        motor_1.publish(twist)
        twist.data = 0 * 0.01744
        motor_2.publish(twist)
        twist.data = 90 * 0.01744
        motor_3.publish(twist)
        twist.data = 0 * 0.01744
        motor_4.publish(twist)
        twist.data = -90 * 0.01744
        motor_5.publish(twist)
        twist.data = 0* 0.01744
        motor_6.publish(twist)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        manipulator_control()
    except rospy.ROSInterruptException: 
        pass
