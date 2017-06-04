#!/usr/bin/env python
import sys
from gui import Window
from threadGUI import ThreadGUI
from ros import RosManager
from PyQt5.QtWidgets import QApplication

if __name__ == "__main__":

	ros_manager = RosManager()

	app = QApplication(sys.argv)
	myGUI = Window(ros_manager)
	myGUI.show()
	
	t = ThreadGUI(myGUI)
	t.daemon=True
	t.start()

	sys.exit(app.exec_())
