#!/usr/bin/env python

import sys

from gui.gui import Window
from gui.threadGUI import ThreadGUI
from ros_manager.ros import RosManager
from PyQt5.QtWidgets import QApplication

if __name__ == "__main__":

	print("Starting the ARIAC arm controller... \n")

	#start the communicating object
	ros_manager = RosManager()

	app = QApplication(sys.argv)

	myGUI = Window(ros_manager)
	myGUI.show()

	t = ThreadGUI(myGUI)
	t.daemon=True
	t.start()

	sys.exit(app.exec_())
