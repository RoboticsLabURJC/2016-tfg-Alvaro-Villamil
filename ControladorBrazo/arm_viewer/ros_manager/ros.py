#!/usr/bin/env python

import rospy
import time
import threading

from threadPublisher import ThreadPublisher
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


class RosManager():

	#defining initial elements for the object
	def __init__(self):
		#used to publish to the arm topic "/ariac/arm/command"
		self.pub = rospy.Publisher("/ariac/arm/command", JointTrajectory, queue_size=10)
		#used to build ros messsages
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
		#used to manage multiple threads, we need one to be
		#constantly sending the position
		self.lock = threading.Lock()
		self.kill_event = threading.Event()
		self.thread = ThreadPublisher(self, self.kill_event)
		self.thread.daemon = True
		#starting the object
		self.start()

	#called from start, get the initial position of the arm
	def get_msg(self, msg):
		self.lock.acquire()
		self.position[0] = msg.position[0]
		self.position[1] = msg.position[1]
		self.position[2] = msg.position[2]
		self.position[3] = msg.position[3]
		self.position[4] = msg.position[4]
		self.position[5] = msg.position[5]
		self.position[6] = msg.position[6]
		self.position[7] = msg.position[7]
		self.lock.release()

	#called from the thread, sends message with last positions
	def send_msg(self):
		msg = JointTrajectory()
		msg.joint_names = self.arm_joint_names
		point = JointTrajectoryPoint()
		self.lock.acquire()
		point.positions = self.position
		self.lock.release()
		point.time_from_start = rospy.Duration(1.0)
		msg.points = [point]
		if not rospy.is_shutdown():
			self.pub.publish(msg)

	#called from the gui (window) each time a value is changed
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


	#called from the gui (window) to get initial values
	def read_elbow(self):
		self.lock.acquire()
		a = self.position[0]
		self.lock.release()
		return a

	def read_linear(self):
		self.lock.acquire()
		a =  self.position[1]
		self.lock.release()
		return a

	def read_shoulder_lift(self):
		self.lock.acquire()
		a =  self.position[2]
		self.lock.release()
		return a

	def read_shoulder_pan(self):
		self.lock.acquire()
		a =  self.position[3]
		self.lock.release()
		return a

	def read_wrist_1(self):
		self.lock.acquire()
		a =  self.position[4]
		self.lock.release()
		return a

	def read_wrist_2(self):
		self.lock.acquire()
		a =  self.position[5]
		self.lock.release()
		return a

	def read_wrist_3(self):
		self.lock.acquire()
		a =  self.position[6]
		self.lock.release()
		return a

	#called at init
	def start(self):

		#creates the rosnode to get and send ros messages
		rospy.init_node('Mando', anonymous=True)

		#recieves rosmessage with initial values of the arm
		#and sends it to get_msg()
		sub = rospy.Subscriber("/ariac/joint_states", JointState, self.get_msg)
		time.sleep(0.1)
		#stop recieving messages
		sub.unregister()

		#Starts the thread. If client is stopped you can not 
		#start again, Threading.Thread raised error
		self.kill_event.clear()
		self.thread.start()

	#called from gui to make a controlled shutdown
	def stop(self):

		print("Exiting the ARIAC arm controller... \n")

		#Stops (Unregisters) the client. If client is stopped 
		#you can not start again, Threading.Thread raised error
		#Also shuts down the rosnode
		self.kill_event.set()
		self.pub.unregister()
		rospy.signal_shutdown("Node Closed")
		




