#!/usr/bin/env python

import rospy
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

def sendmsg():

    msg = JointTrajectory()
    msg.joint_names = [
            'elbow_joint',
            'linear_arm_actuator_joint',
            'shoulder_lift_joint',
            'shoulder_pan_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint',
        ]
    point = JointTrajectoryPoint()
    point.positions = [1.85, 0.35, -0.38, 2.76, 3.67, -1.51, 0.00]
    point.time_from_start = rospy.Duration(1.0)
    msg.points = [point]
    pub = rospy.Publisher('/ariac/arm/command', JointTrajectory, queue_size=10)

    rospy.init_node('mando', anonymous=True)
    if not rospy.is_shutdown():
        print("Sending command: " + str(msg))
        pub.publish(msg)


if __name__ == '__main__':
    try:
        sendmsg()
    except rospy.ROSInterruptException:
        pass

