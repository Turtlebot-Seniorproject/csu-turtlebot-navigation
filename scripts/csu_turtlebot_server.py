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
	    self._ac.wait_for_server()

	    self.gTB = CSUTurtlebotGoal()

        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.serversocket.bind(('', 10888))
        self.serversocket.listen(1)

        while True:
            c, addr = serversocket.accept()
            print("Connection established from " + repr(addr[1]))

            c.send("Connection established")
            room_name = c.recv(10888)
            if room_name == "ERROR: No Room Name":
                print "No room sent, check the client"
            else:
                send_command(room_name, ac, gTB)
                c.close()


    def send_command(room_name, ac, turtlebot_goal):
        location_xy = csu_constants.ROOM_DICTIONARY[room_name]
        turtlebot_goal.locX = location_xy['x']
        turtlebot_goal.locY = location_xy['y']
        ac.send_goal(turtlebot_goal)

if __name__ == '__main__':
    rospy.init_node('csu_turtlebot_server')
    app = wx.App()
    socket_listener = CSU_Turtlebot_Server(None)
    app.MainLoop()
