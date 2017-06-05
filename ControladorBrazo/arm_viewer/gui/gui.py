#!/usr/bin/env python
from PyQt5.QtCore import QPoint, QRect, QSize, Qt, QPointF, QRectF
from PyQt5.QtGui import (QBrush, QConicalGradient, QLinearGradient, QPainter,
		QPainterPath, QPalette, QPen, QPixmap, QPolygon, QRadialGradient, QColor, 
		QTransform, QPolygonF, QKeySequence, QIcon)
from PyQt5.QtWidgets import (QApplication, QCheckBox, QComboBox, QGridLayout,
		QLabel, QSpinBox, QWidget, QPushButton, QSpacerItem, QSizePolicy, QSlider )

class Window(QWidget):

	def __init__(self, ros):

		super(Window, self).__init__()

		self.ros_manager = ros

		mainLayout = QGridLayout()

		#names of each slider
		self.label1 = QLabel("elbow_joint")
		self.label2 = QLabel("linear_arm_actuator_joint")
		self.label3 = QLabel("shoulder_lift_joint")
		self.label4 = QLabel("shoulder_pan_joint")
		self.label5 = QLabel("wrist_1_joint")
		self.label6 = QLabel("wrist_2_joint")
		self.label7 = QLabel("wrist_3_joint")

		#Sliders: creating and limit values of each one
		self.slider1 = QSlider(Qt.Horizontal)
		self.slider1.setMinimum(-628)
		self.slider1.setMaximum(628)
		self.slider1.setValue(float('%.2f'%(self.ros_manager.read_elbow()))*100)
		self.slider2 = QSlider(Qt.Horizontal)
		self.slider2.setMinimum(-210)
		self.slider2.setMaximum(210)
		self.slider2.setValue(float('%.2f'%(self.ros_manager.read_linear()))*100)
		self.slider3 = QSlider(Qt.Horizontal)
		self.slider3.setMinimum(-628)
		self.slider3.setMaximum(628)
		self.slider3.setValue(float('%.2f'%(self.ros_manager.read_shoulder_lift()))*100)
		self.slider4 = QSlider(Qt.Horizontal)
		self.slider4.setMinimum(-628)
		self.slider4.setMaximum(628)
		self.slider4.setValue(float('%.2f'%(self.ros_manager.read_shoulder_pan()))*100)
		self.slider5 = QSlider(Qt.Horizontal)
		self.slider5.setMinimum(-628)
		self.slider5.setMaximum(628)
		self.slider5.setValue(float('%.2f'%(self.ros_manager.read_wrist_1()))*100)
		self.slider6 = QSlider(Qt.Horizontal)
		self.slider6.setMinimum(-628)
		self.slider6.setMaximum(628)
		self.slider6.setValue(float('%.2f'%(self.ros_manager.read_wrist_2()))*100)
		self.slider7 = QSlider(Qt.Horizontal)
		self.slider7.setMinimum(-628)
		self.slider7.setMaximum(628)
		self.slider7.setValue(float('%.2f'%(self.ros_manager.read_wrist_3()))*100)

		#labels for the slider's values
		self.value1 = QLabel('%.2f'%(self.ros_manager.read_elbow()))
		self.value2 = QLabel('%.2f'%(self.ros_manager.read_linear()))
		self.value3 = QLabel('%.2f'%(self.ros_manager.read_shoulder_lift()))
		self.value4 = QLabel('%.2f'%(self.ros_manager.read_shoulder_pan()))
		self.value5 = QLabel('%.2f'%(self.ros_manager.read_wrist_1()))
		self.value6 = QLabel('%.2f'%(self.ros_manager.read_wrist_2()))
		self.value7 = QLabel('%.2f'%(self.ros_manager.read_wrist_3()))

		#size adjustments
		self.ajuste1 = QLabel("                                                                   ")
		mainLayout.addWidget(self.ajuste1,1,1)
		self.ajuste2 = QLabel("           ")
		mainLayout.addWidget(self.ajuste2,1,2)


		#arranging all elements on the window
		mainLayout.addWidget(self.label1,0,0)
		mainLayout.addWidget(self.label2,1,0)
		mainLayout.addWidget(self.label3,2,0)
		mainLayout.addWidget(self.label4,3,0)
		mainLayout.addWidget(self.label5,4,0)
		mainLayout.addWidget(self.label6,5,0)
		mainLayout.addWidget(self.label7,6,0)
		mainLayout.addWidget(self.slider1,0,1)
		mainLayout.addWidget(self.slider2,1,1)
		mainLayout.addWidget(self.slider3,2,1)
		mainLayout.addWidget(self.slider4,3,1)
		mainLayout.addWidget(self.slider5,4,1)
		mainLayout.addWidget(self.slider6,5,1)
		mainLayout.addWidget(self.slider7,6,1)
		mainLayout.addWidget(self.value1,0,2)
		mainLayout.addWidget(self.value2,1,2)
		mainLayout.addWidget(self.value3,2,2)
		mainLayout.addWidget(self.value4,3,2)
		mainLayout.addWidget(self.value5,4,2)
		mainLayout.addWidget(self.value6,5,2)
		mainLayout.addWidget(self.value7,6,2)

		#callbacks to change labels value and send commands to arm
		self.slider1.valueChanged.connect(self.valuechange1)
		self.slider2.valueChanged.connect(self.valuechange2)
		self.slider3.valueChanged.connect(self.valuechange3)
		self.slider4.valueChanged.connect(self.valuechange4)
		self.slider5.valueChanged.connect(self.valuechange5)
		self.slider6.valueChanged.connect(self.valuechange6)
		self.slider7.valueChanged.connect(self.valuechange7)

		#Starting the window
		self.setLayout(mainLayout)
		self.setWindowTitle("ARIAC arm controller")

	#callbacks each time a slider is moved
	def valuechange1(self):
		num = self.slider1.value()
		self.ros_manager.move_elbow(float(num)/100)
		self.value1.setText(str(float(num)/100))

	def valuechange2(self):
		num = self.slider2.value()
		self.ros_manager.move_linear(float(num)/100)
		self.value2.setText(str(float(num)/100))

	def valuechange3(self):
		num = self.slider3.value()
		self.ros_manager.move_shoulder_lift(float(num)/100)
		self.value3.setText(str(float(num)/100))

	def valuechange4(self):
		num = self.slider4.value()
		self.ros_manager.move_shoulder_pan(float(num)/100)
		self.value4.setText(str(float(num)/100))

	def valuechange5(self):
		num = self.slider5.value()
		self.ros_manager.move_wrist_1(float(num)/100)
		self.value5.setText(str(float(num)/100))

	def valuechange6(self):
		num = self.slider6.value()
		self.ros_manager.move_wrist_2(float(num)/100)
		self.value6.setText(str(float(num)/100))

	def valuechange7(self):
		num = self.slider7.value()
		self.ros_manager.move_wrist_3(float(num)/100)
		self.value7.setText(str(float(num)/100))

	#when closing the window makes a controlled shutdown
	def closeEvent(self, event):
		self.ros_manager.stop()
		event.accept() # let the window close




