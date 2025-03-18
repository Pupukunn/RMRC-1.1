#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

received_joy = None
servo1_position = 90
servo2_position = 90

def joy_callback(msg):
    global received_joy
    received_joy = msg
   

if __name__ == "__main__":
    rospy.init_node("joy_controller")
    print(":: Joy Controller is Running ::")

    joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)
    left_servo_pub = rospy.Publisher("/servo1_angle", Int16, queue_size=10)
    right_servo_pub = rospy.Publisher("/servo2_angle", Int16, queue_size=10)
    motor_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            print("Loop is running")
            if received_joy is not None:
                print("Joy received, processing data")
                if received_joy.buttons[4]:
                    servo1_position += 1
                elif received_joy.buttons[6]:
                    servo1_position -= 1
                if received_joy.buttons[5]:
                    servo2_position += 1
                elif received_joy.buttons[7]:
                    servo2_position -= 1
                if received_joy.buttons[1]:
                    servo1_position = 90
                    servo2_position = 90

                servo1_position = max(0, min(servo1_position, 270))
                servo2_position = max(0, min(servo2_position, 270))

                left_servo_pub.publish(servo1_position)
                right_servo_pub.publish(servo2_position)

                cmd = Twist()
                cmd.linear.x = received_joy.axes[1] * 1.0
                cmd.angular.z = received_joy.axes[0] * 1.0
                motor_pub.publish(cmd)

                print(f"Servo1: {servo1_position}, Servo2: {servo2_position}")
                print(f"Motor: Linear={cmd.linear.x}, Angular={cmd.angular.z}")
            else:
                print("No joy data yet")
            rate.sleep()

    except rospy.ROSInterruptException:
        print(":: Joy Controller Shutting Down ::")