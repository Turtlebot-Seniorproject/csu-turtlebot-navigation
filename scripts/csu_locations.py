#!/usr/bin/python

#Description: Cleveland State University - Campus Navigation with Turtlebots
#Purpose: Campus Locations for Turtlebot
#Instructor: Shiqi Zhang
#Group: Adam Thoennes, James Doherty, Steven Eucker, Nicholas Kramer

def getBuildings():
	buildings = ['Fenn Hall']
	return buildings

def getTypes(buildings):
	types = {
		'Fenn Hall':
			['Room','Faculty','Key','General']}
	return types[buildings]

def getLocations(buildings, types):
	locations = {
		'Fenn Hall': {

			'Room': [

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
				'elevator_2'],

			'Faculty': [

				'Shiqi Zhang'],

			'Key': [

				'Foxes Den'],

			'General': [

				'Vending Machines']}}

	return locations[buildings][types]

#[x, y, vertical(0)/horizontal(1) doorway, unpassable(0)/passable(1) doorway]
#x and y are pixel coordinates on map (device)
def getXY(buildings, types, locations):
	locXY = {
		'Fenn Hall': {

			'Room': {

				"132": [324,510,1,0],
				"133A": [380,510,1,0],
				"133B": [432,510,1,0],
				"133C": [508,510,1,0],
				"133D": [536,486,0,0],
				"133": [364,490,0,0],
				"133E_1": [380,470,1,1],
				"129D": [230,598,1,0],
				"131A": [210,522,0,0],
				"130A": [196,644,1,0],
				"130B": [226,730,1,0],
				"133E_2": [456,336,1,1],
				"125C": [594,337,1,0],
				"125_1": [681,325,1,0],
				"127_1": [852,329,1,0],
				"128E": [1102,198,1,0],
				"128D": [1154,282,0,0],
				"102": [1218,600,1,0],
				"elevator_1": [1098,744,1,0],
				"elevator_2": [1030,740,1,0],
				"128": [1251,308,0,0],
				"103": [1353,413,0,0],
				"130": [633,850,0,0],
				"126B": [876,675,1,0],
				"127_2": [887,565,1,0],
				"125": [850,580,0,0],
				"104": [1697,685,1,0],
				"110X": [1731,581,1,0]},

			'Faculty': {

				'Shiqi Zhang': [0, 0, 0, 0]},

			'Key': {

				'Foxes Den': [0, 0, 0, 0]},

			'General': {

				'Vending Machines': [0, 0, 0, 0]}}}

	return locXY[buildings][types][locations]
