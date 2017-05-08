#!/usr/bin/python

#Description: Cleveland State University - Campus Navigation with Turtlebots
#Purpose: Turtlebot GUI
#Instructor: Shiqi Zhang
#Group: Adam Thoennes, James Doherty, Steven Eucker, Nicholas Kramer

try:
    import wx
except ImportError:
    raise ImportError,"The wxPython module is required to run this program."
import wx.media
import rospy
import actionlib
import csu_locations
from std_srvs.srv import *
from actionlib_msgs.msg import *
from csu_turtlebot_navigation.msg import *

class CSU_TurtlebotGUI(wx.Frame):

	def __init__(self, parent, *args, **kwargs):
		self._ac = actionlib.SimpleActionClient('csu_turtlebot_actions', CSUTurtlebotAction)
		self._ac.wait_for_server()

		rospy.Subscriber('csu_turtlebot_result', CSUTurtlebotResult, self.callback)

		self.gTB = CSUTurtlebotGoal()

		wx.Frame.__init__(self, parent, *args, **kwargs)
		self.Center()
		self.SetMinSize(self.GetSize())
		self.parent = parent
		self.TurtlebotGUI()

	def TurtlebotGUI(self):
		#Set incrementers and moving status
		self.iTB = 1
		self.sTB = 0
		self.st = False
		self.moving = False

		#Package Path
		self.pkg_path = '/home/turtlebot/turtlebot_ws/src/csu_turtlebot_navigation'

		#Create Panel
		self.panel = wx.Panel(self)
		self.panel.SetBackgroundColour('white')

		#Video Timer
		self.vt_Turtlebot = wx.Timer(self)
		self.vt_Turtlebot.Start(5000)		# 1 change per 5 seconds

		#Sound Timer
		self.st_Turtlebot = wx.Timer(self)
		self.st_Turtlebot.Start(1000)		# 1 change per 1 second

		#Turtlebot Voice
		self.welcome = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.select = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.movdest = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.continuing = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)
		self.changing = wx.media.MediaCtrl(self.panel, szBackend=wx.media.MEDIABACKEND_GSTREAMER)

		self.welcome.Load(self.pkg_path+'/scripts/Sound/welcome.mp3')
		self.select.Load(self.pkg_path+'/scripts/Sound/select.mp3')
		self.movdest.Load(self.pkg_path+'/scripts/Sound/movdest.mp3')
		self.continuing.Load(self.pkg_path+'/scripts/Sound/continuing.mp3')
		self.changing.Load(self.pkg_path+'/scripts/Sound/changing.mp3')
		
		#Create Image Widgets
		convIMG_CSU = wx.Bitmap(self.pkg_path+'/scripts/Images/CSU.png', wx.BITMAP_TYPE_ANY)
		convIMG_Turtlebot = wx.Bitmap(self.pkg_path+'/scripts/Images/Turtlebot.png', wx.BITMAP_TYPE_ANY)

		self.IMG_CSU = wx.StaticBitmap(self.panel, wx.ID_ANY, convIMG_CSU)
		self.IMG_Turtlebot = wx.StaticBitmap(self.panel, wx.ID_ANY, convIMG_Turtlebot)

		#Create Interactive/Selection Widgets
		building_text = wx.StaticText(self.panel, label='Campus Building:')
		self.building_sel = wx.ComboBox(self.panel, choices=csu_locations.getBuildings())
		self.building_sel.Enable(False)

		type_text = wx.StaticText(self.panel, label='Search by:')
		self.type_sel = wx.ComboBox(self.panel, choices=[])
		self.type_sel.Enable(False)

		location_text = wx.StaticText(self.panel, label='Location:')
		self.location_sel = wx.ComboBox(self.panel, choices=[])
		self.location_sel.Enable(False)

		self.status_text = wx.StaticText(self.panel, label='Welcome to CSU Turtlebot Navigation!')

		self.start = wx.Button(self.panel, label='Start!')
		self.go = wx.Button(self.panel, label='Go!')
		self.go.Hide()

		self.inst_text = wx.StaticText(self.panel, label='~ Tap the touchpad to stop the Turtlebot ~')
		self.inst_text.Hide()

		self.cont = wx.Button(self.panel, label='Continue')
		self.cont.Hide()
		self.change = wx.Button(self.panel, label='Change')
		self.change.Hide()

		#Create BoxSizers
		pBox = wx.BoxSizer(wx.VERTICAL)
		bldBox = wx.BoxSizer(wx.VERTICAL)
		typlocBox = wx.BoxSizer(wx.HORIZONTAL)
		typBox = wx.BoxSizer(wx.VERTICAL)
		locBox = wx.BoxSizer(wx.VERTICAL)
		stopBox = wx.BoxSizer(wx.HORIZONTAL)

		#Set Widgets
		pBox.AddStretchSpacer(prop=1)
		pBox.Add(self.IMG_CSU, 0, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=25)
		pBox.AddStretchSpacer(prop=2)

		bldBox.Add(building_text, 0, flag=wx.RIGHT|wx.ALIGN_CENTER_HORIZONTAL, border=15)
		bldBox.Add(self.building_sel, 0, flag=wx.TOP, border=5)
		pBox.Add(bldBox, 0, flag=wx.ALIGN_CENTER_HORIZONTAL)

		typBox.Add(type_text, 0, flag=wx.LEFT, border=2)
		typBox.Add(self.type_sel, 0, flag=wx.TOP, border=5)
		typlocBox.Add(typBox, 0, flag=wx.ALL, border=5)

		locBox.Add(location_text, 0, flag=wx.LEFT, border=2)
		locBox.Add(self.location_sel, 0, flag=wx.TOP, border=5)
		typlocBox.Add(locBox, 0, flag=wx.ALL, border=5)

		pBox.Add(typlocBox, 0, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=5)
		
		pBox.AddStretchSpacer(prop=2)

		pBox.Add(self.status_text, 0, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)

		pBox.AddStretchSpacer(prop=1)
		
		pBox.Add(self.start, 0, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)
		pBox.Add(self.go, 0, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)

		pBox.Add(self.inst_text, 0, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)

		stopBox.Add(self.cont, 0, flag=wx.RIGHT, border=5)
		stopBox.Add(self.change, 0, flag=wx.LEFT, border=5)
		pBox.Add(stopBox, 0, flag=wx.ALL|wx.ALIGN_CENTER_HORIZONTAL, border=10)
		
		pBox.AddStretchSpacer(prop=2)
		pBox.Add(self.IMG_Turtlebot, 0, flag=wx.BOTTOM|wx.ALIGN_CENTER_HORIZONTAL, border=50)
		pBox.AddStretchSpacer(prop=1)

		#Events
		self.Bind(wx.EVT_TIMER, self.OnPlayTB, self.vt_Turtlebot)
		self.Bind(wx.EVT_TIMER, self.OnPlaySel, self.st_Turtlebot)
		self.building_sel.Bind(wx.EVT_COMBOBOX, self.OnBuildingSel)
		self.type_sel.Bind(wx.EVT_COMBOBOX, self.OnTypeSel)
		self.start.Bind(wx.EVT_BUTTON, self.OnStartSel)
		self.go.Bind(wx.EVT_BUTTON, self.OnGoSel)
		self.panel.Bind(wx.EVT_LEFT_DOWN, self.OnTapSel)
		self.IMG_CSU.Bind(wx.EVT_LEFT_DOWN, self.OnTapSel)
		self.IMG_Turtlebot.Bind(wx.EVT_LEFT_DOWN, self.OnTapSel)
		self.cont.Bind(wx.EVT_BUTTON, self.OnContSel)
		self.change.Bind(wx.EVT_BUTTON, self.OnChangeSel)

		#Set Panel
		self.panel.SetSizer(pBox)
		self.panel.Layout()

	#Campus building combobox event
	def OnBuildingSel(self, event):
		types = csu_locations.getTypes(self.building_sel.GetValue())
		self.type_sel.Enable(True)
		self.type_sel.SetItems(types)

	#Search by combobox event
	def OnTypeSel(self, event):
		locations = csu_locations.getLocations(self.building_sel.GetValue(), self.type_sel.GetValue())
		self.location_sel.Enable(True)
		self.location_sel.SetItems(locations)

	#Start button event
	def OnStartSel(self, event):
		self.st = True
		self.start.Hide()
		self.go.Show()
		self.welcome.Play()
		self.building_sel.Enable(True)
		self.panel.Layout()

	#Go button event
	def OnGoSel(self, event):
		self.movdest.Play()
		building_value = self.building_sel.GetValue()
		type_value = self.type_sel.GetValue()
		location_value = self.location_sel.GetValue()
		if building_value.isspace() == True or type_value.isspace() == True or location_value.isspace() == True \
			or not building_value or not type_value or not location_value:
			self.status_text.SetLabel("Please select a destination.")
			self.panel.Layout()
		else:
			self.locXY = csu_locations.getXY(building_value, type_value, location_value)
			self.setGoTo()
			self.moving = True
			self.status_text.SetLabel("Turtlebot moving to destination...")
			self.building_sel.Enable(False)
			self.type_sel.Enable(False)
			self.location_sel.Enable(False)
			self.go.Hide()
			self.inst_text.Show()
			self.panel.Layout()

	#Tap touchpad event (Turtlebot stops moving and waits)
	def OnTapSel(self, event):
		if self.moving == True:
			self.setWait()
			self.moving = False
			self.sTB = 0
			self.status_text.SetLabel("Would you like to continue or change the destination?")
			self.cont.Show()
			self.change.Show()
			self.inst_text.Hide()
			self.panel.Layout()
		else:
			pass

	#Continue button event
	def OnContSel(self, event):
		self.continuing.Play()
		self.setGoTo()
		self.moving = True
		self.status_text.SetLabel("Turtlebot continuing to destination...")
		self.cont.Hide()
		self.change.Hide()
		self.inst_text.Show()
		self.panel.Layout()

	#Change button event
	def OnChangeSel(self, event):
		self.changing.Play()
		self.moving = False
		self.sTB = 0
		self.status_text.SetLabel("Changing destination.")
		self.building_sel.Enable(True)
		self.type_sel.Enable(True)
		self.location_sel.Enable(True)
		self.cont.Hide()
		self.change.Hide()
		self.inst_text.Hide()
		self.go.Show()
		self.panel.Layout()

	#Play Turtlebot pictures
	def OnPlayTB(self, event):
		self.iTB += 1
		convIMG_Turtlebot = wx.Image(self.pkg_path+'/scripts/Video/Turtlebot/Turtlebot'+str(self.iTB)+'.png', wx.BITMAP_TYPE_ANY)
		self.IMG_Turtlebot.SetBitmap(wx.BitmapFromImage(convIMG_Turtlebot))
		self.panel.Refresh()
		if self.iTB == 2:
			self.iTB = 0

	#Play select a destination voice
	def OnPlaySel(self, event):
		if self.sTB == 20 and self.moving == False:
			self.select.Play()
			self.status_text.SetLabel("Please select a destination.")
			self.panel.Layout()
			self.sTB = 0
		if self.st == True:
			self.sTB += 1

	#Send destination goal to action server
	def setGoTo(self):
		self.gTB.goto = self.locXY
		self._ac.send_goal(self.gTB)

	#Send wait command to action server
	def setWait(self):
		self.gTB.wait = 'true'
		self._ac.send_goal(self.gTB)
		self.gTB.wait = 'false'

	#Result of the destination goal
	def callback(self, data):
		if data.result == 'Success':
			self.moving = False
			self.sTB = 0
			self.building_sel.Enable(True)
			self.type_sel.Enable(True)
			self.location_sel.Enable(True)
			self.inst_text.Hide()
			self.status_text.SetLabel("Destination reached!")
			self.go.Show()
			self.panel.Layout()

if __name__ == '__main__':
	rospy.init_node('csu_turtlebot_gui')
	app = wx.App()
	frame = CSU_TurtlebotGUI(None, title='CSU Indoor Navigation with Turtlebots', size=(768,768))
	frame.Show(True)
	app.MainLoop()
