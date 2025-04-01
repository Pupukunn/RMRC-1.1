import subprocess
import rospy
from std_msgs.msg import Float32

def get_wifi_signal(interface="wlo1"):  # ใช้ wlo1 ตามที่พบ
    try:
        result = subprocess.check_output(["iwconfig", interface], text=True)
        for line in result.splitlines():
            if "Signal level" in line:
                signal = float(line.split("Signal level=")[1].split(" ")[0])
                return signal
        return 0.0  # ถ้าไม่พบค่า
    except Exception as e:
        rospy.logerr(f"Error getting WiFi signal: {e}")
        return 0.0

def wifi_publisher():
    pub = rospy.Publisher('/Hotspot_burritos', Float32, queue_size=10)
    rospy.init_node('wifi_node', anonymous=True)
    rate = rospy.Rate(1)  # อัปเดตทุก 1 วินาที
    while not rospy.is_shutdown():
        signal = get_wifi_signal()
        rospy.loginfo(f"WiFi Signal: {signal} dBm")
        pub.publish(signal)
        rate.sleep()

if __name__ == '__main__':
    try:
        wifi_publisher()
    except rospy.ROSInterruptException:
        pass