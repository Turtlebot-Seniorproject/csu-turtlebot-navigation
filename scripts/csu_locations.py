#!/usr/bin/python

#Description: Cleveland State University - Campus Navigation with Turtlebots
#Purpose: Campus Locations for Turtlebot
#Instructor: Shiqi Zhang
#Group: Adam Thoennes, James Doherty, Steven Eucker, Nicholas Kramer

def getBuildings():
	buildings = ['Fenn Hall']
	return buildings

def getTypes(buildings):
	if buildings == 'Fenn Hall':
		types = ['Room','Faculty','Key','General']
	return types

def getLocations(buildings, types):
	locations = []
	if buildings == 'Fenn Hall':
		if types == 'Room':
			locations = [
				'101',
				'102',
				'103',
				'104']
		if types == 'Faculty':
			locations = [
				'Shiqi Zhang',
				'Yongian Fu']
		if types == 'Key':
			locations = [
				'Foxes Den']
		if types == 'General':
			locations = [
				'Vending Machines']
	return locations

#x and y are pixel coordinates on map (device)
def getXY(buildings, types, locations):
	locXY = {}
	if buildings == 'Fenn Hall':
		if types == 'Room':
			locXY = {
				'101': {'x': 300, 'y': 370, 'z': 0.0, 'w': 1.0},
				'102': {'x': 542, 'y': 310, 'z': 1.0, 'w': 0.0},
				'103': {'x': 500, 'y': 440},
				'104': {'x': 670, 'y': 300}}
		if types == 'Faculty':
			locXY = {
				'Shiqi Zhang': {'x': 400, 'y': 400},
				'Yongian Fu':  {'x': 400, 'y': 400}}
		if types == 'Key':
			locXY = {
				'Foxes Den': {'x': 400, 'y': 400}}
		if types == 'General':
			locXY = {
				'Vending Machines': {'x': 400, 'y': 400}}
	return locXY[locations]
