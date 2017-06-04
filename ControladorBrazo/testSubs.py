#!/usr/bin/env python

import rospy
import time

from sensor_msgs.msg import JointState

def callback(msg):
    print(msg)
    print("\n $$$$$$$$$$$  y esta es la posicion que yo quiero  $$$$$$$$$$ \n")
    print(msg.position[0])
    print(msg.position[1])
    print(msg.position[2])
    print(msg.position[3])
    print(msg.position[4])
    print(msg.position[5])
    print(msg.position[6])
    print(msg.position[7])
    print("\n ***************** y ahora con 2 decimales solo  **************** \n")
    print(float(msg.position[0]))
    print(float(msg.position[1]))
    print(float(msg.position[2]))
    print(float(msg.position[3]))
    print(float(msg.position[4]))
    print(float(msg.position[5]))
    print(float(msg.position[6]))
    print(float(msg.position[7]))

    
def listener():


    rospy.init_node('MandoListen', anonymous=True)

    rospy.Subscriber("/ariac/joint_states", JointState, callback)

    # spin() simply keeps python from exiting until this node is stopped
    # rospy.spin()
    time.sleep(0.02)

if __name__ == '__main__':
    listener()
