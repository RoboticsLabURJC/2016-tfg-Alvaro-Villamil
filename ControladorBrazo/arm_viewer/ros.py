#!/usr/bin/env python

import rospy
import time
import threading

from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class RosManager():

	def __init__(self):
		self.lock = threading.Lock()
		self.pub = rospy.Publisher("/ariac/arm/command", JointTrajectory, queue_size=10)
		self.arm_joint_names = [
			'elbow_joint',
			'linear_arm_actuator_joint',
			'shoulder_lift_joint',
			'shoulder_pan_joint',
			'wrist_1_joint',
			'wrist_2_joint',
			'wrist_3_joint',
			]
		self.position = [0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00]

	def get_msg(self, msg):
		self.lock.acquire()
		self.position = msg.position
		self.lock.release()

	def send_msg(self):
		self.lock.acquire()
		msg = JointTrajectory()
		msg.joint_names = self.arm_joint_names
		point = JointTrajectoryPoint()
		point.positions = self.position
		point.time_from_start = rospy.Duration(1.0)
		msg.points = [point]
		self.lock.release()
		if not rospy.is_shutdown():
			self.pub.publish(msg)

	def move_elbow(self, pos):
		self.lock.acquire()
		self.position[0] = pos
		self.lock.release()
		self.send_msg()

	def move_linear(self, pos):
		self.lock.acquire()
		self.position[1] = pos
		self.lock.release()
		self.send_msg()

	def move_shoulder_lift(self, pos):
		self.lock.acquire()
		self.position[2] = pos
		self.lock.release()
		self.send_msg()

	def move_shoulder_pan(self, pos):
		self.lock.acquire()
		self.position[3] = pos
		self.lock.release()
		self.send_msg()

	def move_wrist_1(self, pos):
		self.lock.acquire()
		self.position[4] = pos
		self.lock.release()
		self.send_msg()

	def move_wrist_2(self, pos):
		self.lock.acquire()
		self.position[5] = pos
		self.lock.release()
		self.send_msg()

	def move_wrist_3(self, pos):
		self.lock.acquire()
		self.position[6] = pos
		self.lock.release()
		self.send_msg()

	def read_elbow(self):
		self.lock.acquire()
		return self.position[0]
		self.lock.release()

	def read_linear(self):
		self.lock.acquire()
		return self.position[1]
		self.lock.release()

	def read_shoulder_lift(self):
		self.lock.acquire()
		return self.position[2]
		self.lock.release()

	def read_shoulder_pan(self):
		self.lock.acquire()
		return self.position[3]
		self.lock.release()

	def read_wrist_1(self):
		self.lock.acquire()
		return self.position[4]
		self.lock.release()

	def read_wrist_2(self):
		self.lock.acquire()
		return self.position[5]
		self.lock.release()

	def read_wrist_3(self):
		self.lock.acquire()
		return self.position[6]
		self.lock.release()




	def start(self):

		rospy.init_node('Mando', anonymous=True)

		rospy.Subscriber("/ariac/joint_states", JointState, self.get_msg)
		time.sleep(0.02)

		print("\n\n **** esto es lo que cojones recibe:\n")
		print(self.position)
		print("\n              ********************************\n")

		rate = rospy.Rate(10) # 10hz
		while not rospy.is_shutdown():
			self.send_msg()
			rate.sleep()


'''   ******** esto es un def start alternativo por si el otro no funciona bien  ********
	def start():

		rospy.init_node('Mando', anonymous=True)

		rospy.Subscriber("/ariac/joint_states", JointState, get_msg)

		self.send_msg()
		# spin() simply keeps python from exiting until this node is stopped
		rospy.spin()

'''









