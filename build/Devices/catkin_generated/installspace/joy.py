#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Int16

# ตัวแปรเก็บค่าจากจอยสติ๊ก
received_joy = None
servo1_position = 90  # เริ่มต้นที่ 90 องศา
servo2_position = 90

def joy_callback(msg):
    global received_joy
    received_joy = msg

if __name__ == "__main__":
    rospy.init_node("joy_controller") 
    rospy.loginfo(":: Joy Controller is Running ::")

    # Subscribe รับค่าจากจอยสติ๊ก
    joy_sub = rospy.Subscriber("/joy", Joy, joy_callback)

    # Publisher สำหรับส่งค่ามุมเซอร์โวไป ROS
    left_servo_pub = rospy.Publisher("/servo1_angle", Int16, queue_size=10)
    right_servo_pub = rospy.Publisher("/servo2_angle", Int16, queue_size=10)

    # Publisher สำหรับส่งค่าควบคุมมอเตอร์ไป ROS
    motor_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    rate = rospy.Rate(10)  # 10Hz

    try:
        while not rospy.is_shutdown():
            if received_joy is not None:
                # **ควบคุม Servo**
                # Servo1 (L1 เพิ่มมุม, L2 ลดมุม)
                if received_joy.buttons[4]:  # L1
                    servo1_position += 1
                elif received_joy.buttons[6]:  # L2
                    servo1_position -= 1

                # Servo2 (R1 เพิ่มมุม, R2 ลดมุม)
                if received_joy.buttons[5]:  # R1
                    servo2_position += 1
                elif received_joy.buttons[7]:  # R2
                    servo2_position -= 1

                # รีเซ็ต Servo1 และ Servo2 กลับไปที่ 90 องศา (ปุ่ม O)
                if received_joy.buttons[1]:  # ปุ่ม O
                    servo1_position = 90
                    servo2_position = 90

                # จำกัดมุมให้อยู่ในช่วง 0 - 270 องศา
                servo1_position = max(0, min(servo1_position, 270))
                servo2_position = max(0, min(servo2_position, 270))

                # ส่งค่าองศาไป ROS
                left_servo_pub.publish(servo1_position)
                right_servo_pub.publish(servo2_position)

                # **ควบคุมมอเตอร์**
                cmd = Twist()
                cmd.linear.x = received_joy.axes[1] * 1.0  # ขึ้น/ลง → ควบคุมเดินหน้า/ถอยหลัง
                cmd.angular.z = received_joy.axes[0] * 1.0  # ซ้าย/ขวา → ควบคุมหมุนซ้าย/ขวา

                motor_pub.publish(cmd)

                rospy.loginfo(f"Servo1: {servo1_position}, Servo2: {servo2_position}")
                rospy.loginfo(f"Motor: Linear={cmd.linear.x}, Angular={cmd.angular.z}")

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo(":: Joy Controller Shutting Down ::")



