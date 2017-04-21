#!/usr/bin/python

#Description: Cleveland State University - Campus Navigation with Turtlebots
#Purpose: Turtlebot Actions
#Instructor: Shiqi Zhang
#Group: Adam Thoennes, James Doherty, Steven Eucker, Nicholas Kramer

import wx
import wx.media
import math
import rospy
import actionlib
import tf
from std_srvs.srv import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from csu_turtlebot_navigation.msg import *

class CSU_TurtlebotActions(wx.Frame):

	def __init__(self, parent, *args, **kwargs):
		self._ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self._ac.wait_for_server()

		self._as = actionlib.SimpleActionServer('csu_turtlebot_actions', CSUTurtlebotAction, self.execute, False)
		self._as.start()

		self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
		#self.pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 10)

		self.gMB = MoveBaseGoal()
		self.listener = tf.TransformListener()
		#self.tp = PoseWithCovarianceStamped()

		#Package Path
		self.pkg_path = '/home/turtlebot/turtlebot_ws/src/csu_turtlebot_navigation'

		wx.Frame.__init__(self, parent, *args, **kwargs)

		#Create Panel
		self.panel = wx.Panel(self)

		#Sounds
		self.odoor = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.odoor.Load('/home/turtlebot/turtlebot_ws/src/csu_turtlebot_navigation/Sound/OpenDoor.mp3')

		self.inten = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.inten.Load(self.pkg_path+'/Sound/In10sec.mp3')

	def execute(self, req):
		#Map dimensions in pixels
		self.mapDimX = 1980
		self.mapDimY = 940
		#Turtlebot origin in meters (logical)
		self.originX = 15.12
		self.originY = 28.65
		#Meters per pixel (1.97 inches per pixel)
		self.mpp = 0.05
		#Door stats
		self.dwait = 0

		self.gTB = req
		
		if self.gTB.wait == 'true':
			self._ac.cancel_all_goals()
			self.dtime.Stop()
			#self.tp.header.frame_id = '/map'
			#self.tp.pose.pose.position.x = 12.0
			#self.tp.pose.pose.orientation.w = 1.0
			#self.pub.publish(self.tp)
		else:
			#self.odoor.Play()
			self.clear_costmap()
			self.dtime = wx.Timer(self)
			self.dtime.Start(1000)

			self.Bind(wx.EVT_TIMER, self.checkDoor, self.dtime)

			#Calculate goal position
			self.gposX = (self.gTB.posX*self.mpp)-self.originX
			self.gposY = ((self.mapDimY-self.gTB.posY)*self.mpp)-self.originY

			#Set goal position/orientation
			self.gMB.target_pose.header.frame_id = '/map'
			self.gMB.target_pose.pose.position.x = self.gposX
			self.gMB.target_pose.pose.position.y = self.gposY
			self.gMB.target_pose.pose.orientation.z = self.gTB.oriZ
			self.gMB.target_pose.pose.orientation.w = self.gTB.oriW
			self._ac.send_goal(self.gMB)

		self._as.set_succeeded()

	def checkDoor(self, event):
		lct = self.listener.getLatestCommonTime('/map', '/base_link')
		cpos, crot = self.listener.lookupTransform('/map', '/base_link', lct)

		#Calculate door position
		dposX = (self.gTB.dXY[0]*self.mpp)-self.originX
		dposY = ((self.mapDimY-self.gTB.dXY[1])*self.mpp)-self.originY
		dpos = [dposX, dposY]

		#Calculate distance between current position and door position
		dist = math.sqrt((cpos[0]-dpos[0])**2 + (cpos[1]-dpos[1])**2)

		if self.dwait == 0:		
			print dist

		if dist > 1.5:
			self.dopen = False

		if dist < 1.5 and self.dopen == False:
			if self.dwait == 0:
				self._ac.cancel_all_goals()
				#self.odoor.Play()
				#self.inten.Play()
			self.dwait += 1
			print self.dwait
			if self.dwait == 10:
				self.dopen = True
				self._ac.send_goal(self.gMB)
				self.dwait = 0

		if self._ac.get_state() == GoalStatus.SUCCEEDED:
			print 'Success!'
			self.dtime.Stop()

if __name__ == '__main__':
	rospy.init_node('csu_turtlebot_actions')
	app = wx.App()
	server = CSU_TurtlebotActions(None)
	app.MainLoop()
