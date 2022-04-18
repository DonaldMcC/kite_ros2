#!/usr/bin/env python3
# from ros wiki for initial testing
import rclpy
from time import sleep
from std_msgs.msg import String, Int16
#remove cv_bridge as not really using anyhow
#from sensor_msgs.msg import Image
#from cv_bridge import CvBridge, CvBridgeError
pub = 0

def init_ros():
    rclpy.init('kite_main', anonymous=True)


def init_motor_msg():
    global pub
    node = rclpy.create_node('motormsg')
    pub = node.create_publisher(Int16, 'motormsg', 10)


def motor_msg(action):
    global pub
    msg=Int16
    msg.data=action
    pub.publish(msg)
    return


if __name__ == '__main__':
    try:
        rclpy.init()
        init_motor_msg()
        rate = 0.1
        while rclpy.ok():
            motor_msg(0)
            sleep(rate)
    except rclpy.ROSInterruptException:
        pass
