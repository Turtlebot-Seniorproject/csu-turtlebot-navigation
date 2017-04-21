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
		#self.pub = rospy.Publisher('initialpose', PoseWithCovarianceStamped, queue_size = 10)

		self.gMB = MoveBaseGoal()
		self.listener = tf.TransformListener()
		#self.tp = PoseWithCovarianceStamped()

		#Package Path
		self.pkg_path = '/home/tp2a/catkin_ws/src/csu_turtlebot_navigation'

		wx.Frame.__init__(self, parent, *args, **kwargs)

		#Create Panel
		self.panel = wx.Panel(self)

		#Turtlebot Voice
		self.welcome = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.open_door = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.thanks = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.next_dest = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.destination = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)

		self.welcome.Load(self.pkg_path+'/scripts/Sound/welcome.mp3')
		self.open_door.Load(self.pkg_path+'/scripts/Sound/open_door.mp3')
		self.thanks.Load(self.pkg_path+'/scripts/Sound/thanks.mp3')
		self.next_dest.Load(self.pkg_path+'/scripts/Sound/next_dest.mp3')
		self.destination.Load(self.pkg_path+'/scripts/Sound/destination.mp3')

	def execute(self, req):
		#Map dimensions in pixels
		self.mapDimX = 1980
		self.mapDimY = 940
		#Turtlebot origin in meters (logical)
		self.originX = 15.12
		self.originY = 28.65
		#Meters per pixel (1.97 inches per pixel)
		self.mpp = 0.05
		#Orientation
		self.zL, self.wL = 1.0, 0.0
		self.zR, self.wR = 0.0, 1.0
		self.zT, self.wT = 0.7, 0.7
		self.zB, self.wB = -0.7, 0.7
		#Door status
		self.clear, self.dwait = 0, 0
		self.dopen = False

		#Action message
		self.gTB = req
		
		#Turtlebot waits
		if self.gTB.wait == 'true':
			self._ac.cancel_all_goals()
			self.dtime.Stop()
			#self.tp.header.frame_id = '/map'
			#self.tp.pose.pose.position.x = 12.0
			#self.tp.pose.pose.orientation.w = 1.0
			#self.pub.publish(self.tp)
		#Turtlebot sets destination
		else:
			self.welcome.Play()
			self.clear_costmap()

			#Timer to periodically check door status
			self.dtime = wx.Timer(self)
			self.dtime.Start(1000)

			self.Bind(wx.EVT_TIMER, self.checkDoor, self.dtime)

			#Calculate goal position
			self.gotoX = (self.gTB.goto[0]*self.mpp)-self.originX
			self.gotoY = ((self.mapDimY-self.gTB.goto[1])*self.mpp)-self.originY
			self.goto = [self.gotoX, self.gotoY]

			#Set goal position/orientation
			self.gMB.target_pose.header.frame_id = '/map'
			self.gMB.target_pose.pose.position.x = self.gotoX
			self.gMB.target_pose.pose.position.y = self.gotoY
			self.gMB.target_pose.pose.orientation.z = self.zR
			self.gMB.target_pose.pose.orientation.w = self.wR

			self.clear += 1

		self._as.set_succeeded()

	def checkDoor(self, event):
		#Wait for costmap clear
		if self.clear == 1:
			self._ac.send_goal(self.gMB)
			self.clear = 0

		#Get current turtlebot position
		self.lct = self.listener.getLatestCommonTime('/map', '/base_link')
		self.cpos, self.crot = self.listener.lookupTransform('/map', '/base_link', self.lct)

		#Calculate door position
		self.dposX = (self.gTB.dXY[0]*self.mpp)-self.originX
		self.dposY = ((self.mapDimY-self.gTB.dXY[1])*self.mpp)-self.originY
		self.dpos = [self.dposX, self.dposY]

		#Calculate distance between current position and door position
		self.dist = math.sqrt((self.cpos[0]-self.dpos[0])**2 + (self.cpos[1]-self.dpos[1])**2)

		if self.dwait == 0:		
			print self.dist

		#Door initially closed
		if self.dist > 1.5:
			self.dopen = False

		#Wait for door to open
		if self.dist < 1.5 and self.dopen == False and self.goto != self.dpos:
			if self.dwait == 0:
				self._ac.cancel_all_goals()
				self.open_door.Play()
			self.dwait += 1
			print self.dwait
			if self.dwait == 10:
				self.thanks.Play()
				self.dopen = True
				self._ac.send_goal(self.gMB)
				self.dwait = 0

		#Reposition and face the destination
		if self.dist < 3.0 and self.goto == self.dpos and self._ac.get_state() == GoalStatus.ACTIVE:
			if self.gTB.goto[2] == 0 and self.cpos[0] > self.dpos[0]:
				self.gMB.target_pose.pose.position.x = self.gotoX+1.5
				self.gMB.target_pose.pose.orientation.z = self.zL
				self.gMB.target_pose.pose.orientation.w = self.wL
			if self.gTB.goto[2] == 0 and self.cpos[0] < self.dpos[0]:
				self.gMB.target_pose.pose.position.x = self.gotoX-1.5
				self.gMB.target_pose.pose.orientation.z = self.zR
				self.gMB.target_pose.pose.orientation.w = self.wR
			if self.gTB.goto[2] == 1 and self.cpos[1] > self.dpos[1]:
				self.gMB.target_pose.pose.position.y = self.gotoY+1.5
				self.gMB.target_pose.pose.orientation.z = self.zT
				self.gMB.target_pose.pose.orientation.w = self.wT
			if self.gTB.goto[2] == 1 and self.cpos[1] < self.dpos[1]:
				self.gMB.target_pose.pose.position.y = self.gotoY-1.5
				self.gMB.target_pose.pose.orientation.z = self.zB
				self.gMB.target_pose.pose.orientation.w = self.wB
			self._ac.send_goal(self.gMB)

		#Destination reached
		if self._ac.get_state() == GoalStatus.SUCCEEDED:
			self.destination.Play()
			#self.next_dest.Play()
			print 'Success!'
			self.dtime.Stop()

if __name__ == '__main__':
	rospy.init_node('csu_turtlebot_actions')
	app = wx.App()
	server = CSU_TurtlebotActions(None)
	app.MainLoop()
