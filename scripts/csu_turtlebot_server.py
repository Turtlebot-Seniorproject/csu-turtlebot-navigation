#!/usr/bin/python

import rospy
import wx
import socket
import actionlib
import csu_constants
from csu_turtlebot_navigation.msg import *


class CSU_Turtlebot_Server(wx.Frame):
    
    def __init__(self, parent, *args, **kwargs):
        self._ac = actionlib.SimpleActionClient('csu_turtlebot_actions', CSUTurtlebotAction)
	print "wating for ac"
	self._ac.wait_for_server()

	self.gTB = CSUTurtlebotGoal()

        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.bind(('', 10888))
	print "I am listening"
        self.serversocket.listen(5)

        while True:
            c, addr = self.serversocket.accept()
            print("Connection established from " + repr(addr[1]))

            c.send("Connection established")
            room_name = c.recv(10888)
            # do some real validation here
            if room_name == "ERROR":
                print "No room sent, check the client"
            elif room_name == "WAIT":
                self.gTB.wait = 'true'
                self._ac.send_goal(self.gTB)
                self.gTB.wait = 'false'
            else:
                self.send_command(room_name)


    def send_command(self, room_name="ERROR"):
        self.gTB.goto = csu_constants.ROOM_DICTIONARY[room_name]
        self._ac.send_goal(self.gTB)

if __name__ == '__main__':
    rospy.init_node('csu_turtlebot_server')
    app = wx.App()
    socket_listener = CSU_Turtlebot_Server(None)
    app.MainLoop()
