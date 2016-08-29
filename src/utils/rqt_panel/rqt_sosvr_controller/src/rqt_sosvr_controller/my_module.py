import os
import rospkg
import rospy

from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtGui import QWidget
from python_qt_binding.QtGui import QIcon
from threading import Thread
from time import sleep

class MyPlugin(Plugin):

	def __init__(self, context):
		super(MyPlugin, self).__init__(context)
		# Give QObjects reasonable names
		self.setObjectName('MyPlugin')
		rp = rospkg.RosPack()

		# Process standalone plugin command-line arguments
		from argparse import ArgumentParser
		parser = ArgumentParser()
		# Add argument(s) to the parser.
		parser.add_argument("-q", "--quiet", action="store_true",
					dest="quiet",
					help="Put plugin in silent mode")
		args, unknowns = parser.parse_known_args(context.argv())
		if not args.quiet:
			print 'arguments: ', args
			print 'unknowns: ', unknowns
		
		# Create QWidget
		self._widget = QWidget()
		# Get path to UI file which is a sibling of this file
		# in this example the .ui and .py file are in the same folder
		ui_file = os.path.join(rp.get_path('rqt_sosvr_controller'), 'resource', 'MyPlugin.ui')
		# Extend the widget with all attributes and children from UI file
		loadUi(ui_file, self._widget)
		# Give QObjects reasonable names
		self._widget.setObjectName('MyPluginUi')
		# Show _widget.windowTitle on left-top of each plugin (when
		# it's set in _widget). This is useful when you open multiple
		# plugins at once. Also if you open multiple instances of your
		# plugin at once, these lines add number to make it easy to
		# tell from pane to pane.
		if context.serial_number() > 1:
			self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
		# Add widget to the user interface

		robot_names = ['robot1', 'robot2', 'robot3', 'robot4']
		self._widget.robot_box.addItems(robot_names)

		self._widget.controlUp.setIcon(QIcon.fromTheme('up'))
		self._widget.controlBottom.setIcon(QIcon.fromTheme('down'))
		self._widget.controlRight.setIcon(QIcon.fromTheme('next'))
		self._widget.controlLeft.setIcon(QIcon.fromTheme('back'))
		
		self._widget.buttonOn.setIcon(QIcon.fromTheme('player_play'))
		self._widget.buttonOff.setIcon(QIcon.fromTheme('gtk-stop'))

		self._widget.controlUp.clicked.connect(self.controlSignalUp) # 1 Up
		self._widget.controlRight.clicked.connect(self.controlSignalRight) # 2 Right
		self._widget.controlBottom.clicked.connect(self.controlSignalBottom) # 3 Bottom
		self._widget.controlLeft.clicked.connect(self.controlSignalLeft) # 4 Down

		self._widget.buttonOn.clicked.connect(self.turnRobotOn)
		self._widget.buttonOff.clicked.connect(self.turnRobotOff)
		
		ss_pub1 = rospy.Publisher('robot1/cmd_vel', Twist, queue_size=10)
		ss_pub2 = rospy.Publisher('robot2/cmd_vel', Twist, queue_size=10)
		ss_pub3 = rospy.Publisher('robot3/cmd_vel', Twist, queue_size=10)
		ss_pub4 = rospy.Publisher('robot4/cmd_vel', Twist, queue_size=10)
		self._pubs = [ss_pub1, ss_pub2, ss_pub3, ss_pub4]
		
		context.add_widget(self._widget)
		self.subscribeToJoy()

	def checkJoyData(self, data):
		if (data.buttons[7] == 1):
			nextIndex = self.getNextIndex()
			self._widget.robot_box.setCurrentIndex(nextIndex)
			#self._widget.textBrowser.moveCursor(QtGui.QTextCursor.End)

	def publishCmdVel(self, data):
		self._pubs[int(str(self._widget.robot_box.currentText())[5:])-1].publish(data)

	def getNextIndex(self):
		return int(str(self._widget.robot_box.currentText())[5:]) % 4
		
	def shutdown_plugin(self):
		# TODO unregister all publishers here
		pass
	def save_settings(self, plugin_settings, instance_settings):
		# TODO save intrinsic configuration, usually using:
		# instance_settings.set_value(k, v)
		pass
	def restore_settings(self, plugin_settings, instance_settings):
		# TODO restore intrinsic configuration, usually using:
		# v = instance_settings.value(k)
		pass
		#def trigger_configuration(self):
		# Comment in to signal that the plugin has a way to configure
		# This will enable a setting button (gear icon) in each dock widget title bar
		# Usually used to open a modal configuration dialog

	def controlSignalUp(self):
		print('Directing {0} to up'.format(str(self._widget.robot_box.currentText())))

	def controlSignalRight(self):
		print('Directing {0} to right'.format(str(self._widget.robot_box.currentText())))

	def controlSignalBottom(self):
		print('Directing {0} to bottom'.format(str(self._widget.robot_box.currentText())))

	def controlSignalLeft(self):
		print('Directing {0} to left'.format(str(self._widget.robot_box.currentText())))

	def turnRobotOn(self):
		print('Turning {0} on'.format(str(self._widget.robot_box.currentText())))
		
	def subscribeToJoy(self):
		rospy.Subscriber('cmd_vel', Twist, self.publishCmdVel)
		rospy.Subscriber('joy', Joy, self.checkJoyData)

	def turnRobotOff(self):
		print('Turning {0} off'.format(str(self._widget.robot_box.currentText())))
	
