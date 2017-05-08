#!/usr/bin/python

#Description: Cleveland State University - Campus Navigation with Turtlebots
#Purpose: Turtlebot Actions
#Instructor: Shiqi Zhang
#Group: Adam Thoennes, James Doherty, Steven Eucker, Nicholas Kramer

try:
    import wx
except ImportError:
    raise ImportError,"The wxPython module is required to run this program."
import wx.media
import math
import rospy
import actionlib
import tf
import csu_constants
from std_srvs.srv import *
from actionlib_msgs.msg import *
from move_base_msgs.msg import *
from geometry_msgs.msg import *
from csu_turtlebot_navigation.msg import *

class CSU_TurtlebotActions(wx.Frame):

	def __init__(self, parent, *args, **kwargs):
		#Set action client
		self._ac = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		self._ac.wait_for_server()

		#Set action server
		self._as = actionlib.SimpleActionServer('csu_turtlebot_actions', CSUTurtlebotAction, self.execute, False)
		self._as.start()

		self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
		self.rTB = rospy.Publisher('csu_turtlebot_result', CSUTurtlebotResult, queue_size=10)

		self.gMB = MoveBaseGoal()
		self.listener = tf.TransformListener()

		#Package Path
		self.pkg_path = '/home/turtlebot/turtlebot_ws/src/csu_turtlebot_navigation'

		wx.Frame.__init__(self, parent, *args, **kwargs)

		#Create Panel
		self.panel = wx.Panel(self)

		#Turtlebot Voice Commands
		self.open = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.thanks = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.stop = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.left = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.right = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.ahead = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.contq = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.repos = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.reached = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		
		self.open.Load(self.pkg_path+'/scripts/Sound/open.mp3')
		self.thanks.Load(self.pkg_path+'/scripts/Sound/thanks.mp3')
		self.stop.Load(self.pkg_path+'/scripts/Sound/stop.mp3')
#		self.left.Load(self.pkg_path+'/scripts/Sound/left.mp3')
#		self.right.Load(self.pkg_path+'/scripts/Sound/right.mp3')
#		self.ahead.Load(self.pkg_path+'/scripts/Sound/ahead.mp3')
		self.contq.Load(self.pkg_path+'/scripts/Sound/continue.mp3')
		self.repos.Load(self.pkg_path+'/scripts/Sound/repos.mp3')
		self.reached.Load(self.pkg_path+'/scripts/Sound/reached.mp3')

		#Turtlebot Voice Rooms
#		self.r103 = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.r125 = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.r125C = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.r127 = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.r128 = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
#		self.r128D = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)

#		self.r103.Load(self.pkg_path+'/scripts/Sound/r103.mp3')
#		self.r125.Load(self.pkg_path+'/scripts/Sound/r125.mp3')
#		self.r125C.Load(self.pkg_path+'/scripts/Sound/r125C.mp3')
#		self.r127.Load(self.pkg_path+'/scripts/Sound/r127.mp3')
#		self.r128.Load(self.pkg_path+'/scripts/Sound/r128.mp3')
#		self.r128D.Load(self.pkg_path+'/scripts/Sound/r128D.mp3')

		#Map dimensions in pixels
		self.mapDimX = 1980
		self.mapDimY = 940
		#Turtlebot origin in meters (logical)
		self.originX = 15.12
		self.originY = 28.65
		#Meters per pixel (1.97 inches per pixel)
		self.mpp = 0.05
		#Orientation values (left, right, top, bottom)
		self.zL, self.wL = 1.0, 0.0
		self.zR, self.wR = 0.0, 1.0
		self.zT, self.wT = 0.7, 0.7
		self.zB, self.wB = -0.7, 0.7
		#Door status
		self.clear, self.dwait = 0, 0
		self.dopen = False
		self.drepos = False
		#Nearest distance, name, position, orientation, and current position
		self.ndist = 1000
		self.nname = "no nearest yet"
		self.npos = [0, 0, 0, 0]
		self.nori = [0, 0, 0, 0]
		self.ncpos = [0, 0, 0]

#		self.vtime = 0
#		self.speak = True

	#Executes actions sent from GUI
	#Actions - Pauses the Turtlebot or sets a destination
	def execute(self, req):
		#Action message
		self.gTB = req

		#Turtlebot waits
		if self.gTB.wait == 'true':
			self._ac.cancel_all_goals()
			self.stop.Play()
			self.dtime.Stop()

		#Turtlebot sets destination
		else:
			#Clear costmap before moving to destination
			self.clear_costmap()

			#Timer to periodically check on nearest door
			self.dtime = wx.Timer(self)
			self.dtime.Start(1000)

			self.Bind(wx.EVT_TIMER, self.nearDoor, self.dtime)

			#Convert goal position from pixel coordinates to meter coordinates (goto = goal position)
			self.gotoX = (self.gTB.goto[0]*self.mpp)-self.originX
			self.gotoY = ((self.mapDimY-self.gTB.goto[1])*self.mpp)-self.originY
			self.goto = [self.gotoX, self.gotoY, self.gTB.goto[2], self.gTB.goto[3]]

			#Set goal position/orientation
			self.gMB.target_pose.header.frame_id = '/map'
			self.gMB.target_pose.pose.position.x = self.gotoX
			self.gMB.target_pose.pose.position.y = self.gotoY
			self.gMB.target_pose.pose.orientation.z = self.zR
			self.gMB.target_pose.pose.orientation.w = self.wR

			self.clear += 1

		self._as.set_succeeded()

	#Determines nearest door to Turtlebot by checking every second
	#Actions - Waits 10 seconds for door to open if passing through or repositions in front of door if destination
	def nearDoor(self, event):
		#Wait a second for costmap to clear then send goal
		if self.clear == 1:
			self._ac.send_goal(self.gMB)
			self.drepos = False
			self.clear = 0

		#Get current turtlebot position (cpos = current position, cori = current orientation)
		self.lct = self.listener.getLatestCommonTime('/map', '/base_link')
		self.cpos, self.cori = self.listener.lookupTransform('/map', '/base_link', self.lct)

		#Cycle through list of doors to determine the nearest door to Turtlebot
		for name, door in csu_constants.ROOM_DICTIONARY.iteritems():

			#Convert door position from pixel coordinates to meter coordinates (dpos = door position)
			self.dposX = (door[0]*self.mpp)-self.originX
			self.dposY = ((self.mapDimY-door[1])*self.mpp)-self.originY
			self.dpos = [self.dposX, self.dposY, door[2], door[3]]
			
			#Calculate distance between current position and door position
			self.dist = math.sqrt((self.cpos[0]-self.dpos[0])**2 + (self.cpos[1]-self.dpos[1])**2)

			#Sets all the nearest values if door happens to be nearest door
			if self.dist < self.ndist:
				self.ndist = self.dist
				self.nname = name
				self.npos = self.dpos
				self.nori = self.cori
				self.ncpos = self.cpos

#Turtlebot determines and voices the direction of nearest room (left, right, ahead) and the room
#Not Complete----------------------------------------------------------------------------------------
#		if self.ndist > 2.0 and self.speak == False:
#			self.vtime = 0
#			self.speak = True

#		if self.ndist < 2.0 and self.speak == True and self.dopen == False and self.npos[3] == 0:
#			if self.nori[3] > 0.7 and self.nori[3] <= 1.0 and self.vtime == 0:
#				if self.ncpos[1] < self.npos[1] and self.npos[2] == 1:
#					self.left.Play()
#				elif self.ncpos[1] > self.npos[1] and self.npos[2] == 1:
#					self.right.Play()
#				else:
#					self.ahead.Play()
#			else:
#				if self.vtime == 0:
#					if self.ncpos[1] < self.npos[1] and self.npos[2] == 1:
#						self.right.Play()
#					elif self.ncpos[1] > self.npos[1] and self.npos[2] == 1:
#						self.left.Play()
#					else:
#						self.ahead.Play()
#			if self.vtime == 2:
#				self.vRooms()
#				self.speak = False
#			if self.vtime < 2:
#				self.vtime += 1
#----------------------------------------------------------------------------------------------------

		#Assume door initially closed when farther than 1.5 meters from door
		if self.ndist > 1.5 and self.dopen == True:
			self.dopen = False

		#Wait 10 seconds for door to open when within 1.5 meters of door
		if self.ndist < 1.5 and self.dopen == False and self.goto != self.npos and self.npos[3] == 1:
			if self.dwait == 0:
				self._ac.cancel_all_goals()
				self.open.Play()
			self.dwait += 1
			if self.dwait == 10:
				self.thanks.Play()
				self.dopen = True
				self._ac.send_goal(self.gMB)
				self.dwait = 0

		#Reposition 1.5 meters from and face the destination when within 4.0 meters from destination
		if self.ndist < 4.0 and self.goto == self.npos and self.drepos == False and self._ac.get_state() == GoalStatus.ACTIVE:
			self.repos.Play()
			if self.gTB.goto[2] == 0 and self.cpos[0] > self.npos[0]:
				self.gMB.target_pose.pose.position.x = self.gotoX+1.5
				self.gMB.target_pose.pose.orientation.z = self.zL
				self.gMB.target_pose.pose.orientation.w = self.wL
			elif self.gTB.goto[2] == 0 and self.cpos[0] < self.npos[0]:
				self.gMB.target_pose.pose.position.x = self.gotoX-1.5
				self.gMB.target_pose.pose.orientation.z = self.zR
				self.gMB.target_pose.pose.orientation.w = self.wR
			elif self.gTB.goto[2] == 1 and self.cpos[1] > self.npos[1]:
				self.gMB.target_pose.pose.position.y = self.gotoY+1.5
				self.gMB.target_pose.pose.orientation.z = self.zT
				self.gMB.target_pose.pose.orientation.w = self.wT
			elif self.gTB.goto[2] == 1 and self.cpos[1] < self.npos[1]:
				self.gMB.target_pose.pose.position.y = self.gotoY-1.5
				self.gMB.target_pose.pose.orientation.z = self.zB
				self.gMB.target_pose.pose.orientation.w = self.wB
			else:
				pass
			self._ac.send_goal(self.gMB)
			self.drepos = True

		#Destination reached
		if self._ac.get_state() == GoalStatus.SUCCEEDED:
			self.reached.Play()
			self.rTB.publish('Success')
			self.dtime.Stop()

		#Resets nearest distance
		self.ndist = 1000

#Turtlebot voices the nearest room
#Not Complete-------------------------------
#	def vRooms(self):
#		if self.nname == '103':
#			self.r103.Play()
#		elif self.nname == '125_1':
#			self.r125.Play()
#		elif self.nname == '125C':
#			self.r125C.Play()
#		elif self.nname == '127_1':
#			self.r127.Play()
#		elif self.nname == '128D':
#			self.r128D.Play()
#		elif self.nname == '128':
#			self.r128.Play()
#		else:
#			pass
#-------------------------------------------

if __name__ == '__main__':
	rospy.init_node('csu_turtlebot_actions')
	app = wx.App()
	server = CSU_TurtlebotActions(None)
	app.MainLoop()
