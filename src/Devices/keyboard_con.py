#!/usr/bin/env python3
import rospy
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
from pynput import keyboard as kb

pressed_keys = set()
def on_press(key):
    try:
        pressed_keys.add(key.char)
    except AttributeError:
        if key == kb.Key.space:  # ตรวจจับ Spacebar
            pressed_keys.add('space')
    

def on_release(key):
    try:
        pressed_keys.remove(key.char)
    except (AttributeError, KeyError):
        if key == kb.Key.space:
            pressed_keys.remove('space')

listener = kb.Listener(on_press=on_press, on_release=on_release)
listener.start()

# ตัวแปรสำหรับเซอร์โว
servo1_position = 90
servo2_position = 90
# ตัวแปรสำหรับเซอร์โวที่เเบน
arm1_position = 90
arm2_position = 90
arm3_position = 90
arm4_position = 90
arm5_position = 90
motorL = 70
motorR = 70
if __name__ == "__main__":
    rospy.init_node("keyboard_controller")
    print(":: Keyboard Controller is Running ::")
    print("Controls:")
    print("  W/A/S/D: Move robot (Linear/Angular)")
    print("  Q/E: Adjust Servo 1 (Left)")
    print("  R/T: Adjust Servo 2 (Right)")
    print("  Space: Reset Servos to 90°")

    # Publisher สำหรับส่งค่ามุมเซอร์โวและการเคลื่อนที่
    left_servo_pub = rospy.Publisher("/servo1_angle", Int16, queue_size=10)
    right_servo_pub = rospy.Publisher("/servo2_angle", Int16, queue_size=10)

    arm1_pub = rospy.Publisher("/arm1", Int16, queue_size=10)
    arm2_pub = rospy.Publisher("/arm2", Int16, queue_size=10)
    arm3_pub = rospy.Publisher("/arm3", Int16, queue_size=10)
    arm4_pub = rospy.Publisher("/arm4", Int16, queue_size=10)
    arm5_pub = rospy.Publisher("/arm5", Int16, queue_size=10)

    motorL_pub = rospy.Publisher("/motorL", Int16, queue_size=10)
    motorR_pub = rospy.Publisher("/motorR", Int16, queue_size=10)
    rate = rospy.Rate(10)  # 10Hz

    try:
        while not rospy.is_shutdown():

            # ควบคุมการเคลื่อนที่
            if 'w' in pressed_keys:  # เดินหน้า
                motorL += 1
                motorR += 1
            elif 's' in pressed_keys:  # ถอยหลัง
                motorL-= 1
                motorR -= 1
            if 'a' in pressed_keys:  # หมุนซ้าย
                motorL -= 1
                motorR += 1
            elif 'd' in pressed_keys:  # หมุนขวา
                motorL += 1
                motorR -= 1
            # ควบคุมการเคลื่อนที่เเบบข้างเดียว
            if '1' in pressed_keys:  # มอเตอร์ด้านซ้ายขยับไปข้างหน้า
                motorL += 1
            elif 'q' in pressed_keys:  # มอเตอร์ด้านซ้ายขยับไปข้างหลัง
                motorL -= 1
            elif '3' in pressed_keys:  # มอเตอร์ด้านขวาขยับไปข้างหน้า
                motorR += 1
            elif 'e' in pressed_keys:  # มอเตอร์ด้านขวาขยับไปข้างหลัง
                motorR -= 1

            # ควบคุม Servo 1 (แทน buttons[4] และ [6])
            if ('z') in pressed_keys:  # เพิ่มมุม Servo 1
                servo1_position += 1
            elif ('x') in pressed_keys:  # ลดมุม Servo 1
                servo1_position -= 1

            # ควบคุม Servo 2 (แทน buttons[5] และ [7])
            if ('c') in pressed_keys:  # เพิ่มมุม Servo 2
                servo2_position += 1
            elif ('v') in pressed_keys:  # ลดมุม Servo 2
                servo2_position -= 1

            # รีเซ็ต Servo (แทน buttons[1])
            if 'space' in pressed_keys:  # รีเซ็ตทั้งสองเซอร์โว
                servo1_position = 90
                servo2_position = 90

            if ('y') in pressed_keys:  # เพิ่มมุม arm1
                arm1_position += 1
            elif ('h') in pressed_keys:  # ลดมุม arm1
                arm1_position -= 1

            if ('u') in pressed_keys:  # เพิ่มมุม arm2
                arm2_position += 1
            elif ('j') in pressed_keys:  # ลดมุม arm2
                arm2_position -= 1

            if ('i') in pressed_keys:  # เพิ่มมุม arm3
                arm3_position += 1
            elif ('k') in pressed_keys:  # ลดมุม arm3
                arm3_position -= 1

            if ('o') in pressed_keys:   # เพิ่มมุม arm4
                arm4_position += 1
            elif ('l') in pressed_keys:  # ลดมุม arm4
                arm4_position -= 1

            if ('p') in pressed_keys:  # เพิ่มมุม arm5
                arm5_position += 1
            elif (';' in pressed_keys):  # ลดมุม arm5
                arm5_position -= 1
            
            # จำกัดมุมให้อยู่ในช่วง 0-270
            servo1_position = max(0, min(servo1_position, 270))
            servo2_position = max(0, min(servo2_position, 270))
            
            arm1_position = max(0, min(arm1_position, 180))
            arm2_position = max(0, min(arm2_position, 180))
            arm3_position = max(0, min(arm3_position, 180))
            arm4_position = max(0, min(arm4_position, 180))
            arm5_position = max(0, min(arm5_position, 180))

            motorL = max(70, min(motorL, 100))
            motorR = max(70, min(motorR, 100))
            # Publish ค่าไปยัง ROS
            left_servo_pub.publish(servo1_position)
            right_servo_pub.publish(servo2_position)

            arm1_pub.publish(arm1_position)
            arm2_pub.publish(arm2_position)
            arm3_pub.publish(arm3_position)
            arm4_pub.publish(arm4_position)
            arm5_pub.publish(arm5_position)

            motorL_pub.publish(motorL)
            motorR_pub.publish(motorR)
            # แสดงผลในเทอร์มินัลเพื่อ debug
            print(f"Servo1: {servo1_position}, Servo2: {servo2_position}")
            print(f"Arm1: {arm1_position}, Arm2: {arm2_position},Arm3: {arm3_position},Arm4: {arm4_position},Arm5: {arm5_position}")
            print(f"Motor_left: {motorL},Motor_right: {motorR}")

            rate.sleep()

    except rospy.ROSInterruptException:
        print(":: Keyboard Controller Shutting Down ::")
    except Exception as e:
        print(f"Error: {e}")