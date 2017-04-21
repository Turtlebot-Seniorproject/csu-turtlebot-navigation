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
				'102',
				'103',
				'104',
				'110X',
				'125',
				'125_1',
				'125C',
				'126B',
				'127_1',
				'127_2',
				'128',
				'128D',
				'128E',
				'129D',
				'130',
				'130A',
				'130B',
				'131A',
				'132',
				'133',
				'133A',
				'133B',
				'133C',
				'133D',
				'133E_1',
				'133E_2',
				'elevator_1',
				'elevator_2']
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
				"132": [324,510,1],
				"133A": [380,510,1],
				"133B": [432,510,1],
				"133C": [508,510,1],
				"133D": [536,486,0],
				"133": [364,490,0],
				"133E_1": [380,470,1],
				"129D": [230,598,1],
				"131A": [210,522,0],
				"130A": [196,644,1],
				"130B": [226,730,1],
				"133E_2": [456,336,1],
				"125C": [592,338,1],
				"125_1": [668,326,1],
				"127_1": [886,328,1],
				"128E": [1102,198,1],
				"128D": [1154,282,0],
				"102": [1218,600,1],
				"elevator_1": [1098,744,1],
				"elevator_2": [1030,740,1],
				"128": [1251,308,0],
				"103": [1353,413,0],
				"130": [633,850,0],
				"126B": [876,675,1],
				"127_2": [887,565,1],
				"125": [850,580,0],
				"104": [1697,685,1],
				"110X": [1731,581,1]}
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
