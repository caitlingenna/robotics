from connectsonar import *
from matplotlib import pyplot as plt
from math import *

#initialize map
GRID_SIZE = 1000
grid = [0] * GRID_SIZE
for i in range(0, GRID_SIZE):
	grid[i] = [0] * GRID_SIZE

def readSonar():
	for s in range(0, robot.getNumSonar()):
		x = robot.getSonarReading(s).getX()
		y = robot.getSonarReading(s).getY()
		# restrict coordinates to -10,000 to 10,000		
		if (x > -10000 and x < 10000) and (y > -10000 and y < 10000):
			i = int((x + 10000)/20)
			j = int((y + 10000)/20)
			grid[i][j] = grid[i][j] + 1

def difference(i, goalHeading):
	difference = 0
	if((goalHeading < 180 and i < 180) or (goalHeading > 180 and i > 180)):
		difference = abs(goalHeading - i)
	elif(i < 180):
		difference = 360 - goalHeading + i
	else:
		difference = 360 - i + goalHeading 
	return difference

def VFH(GOALX, GOALY, grid, GRID_SIZE):	
	#get sonar readings
	counter = 0
	while(counter < 1000):
		readSonar()
		counter = counter + 1
	
	histo = [0] * 360
	#locate grid coordinates of robot
	xRobot = robot.getX()
	yRobot = robot.getY()
	iRobot = int((xRobot + 10000)/20) 
	jRobot = int((yRobot + 10000)/20)

	# define boundaries for active grid, make sure they aren't out of bounds of grid
	iActiveMin = iRobot - 100
	if(iActiveMin < 0):
		iActiveMin = 0
	iActiveMax = iRobot + 100
	if(iActiveMax > 1000):
		iActiveMax = 1000	
	jActiveMin = jRobot - 100
	if(jActiveMin < 0):
		jActiveMin = 0
	jActiveMax = jRobot + 100
	if(jActiveMax > 1000):
		jActiveMax = 1000
	#determine values for vfh equation
	a = 250
	b = 1
	c = 0
	d = 0
	angle = 0
	for i in range (iActiveMin, iActiveMax):
		for j in range (jActiveMin, jActiveMax):
			c = (grid[i][j])
			if (c > 20): #cap certainty values at 20
				c = 20
		d = sqrt(((i - iRobot)**2) + ((j - jRobot)**2)) #distance formula
		angle = atan2((i-iRobot), (j-jRobot)) #angle between robot and cell
		angle = degrees(angle)
		#calculate value, add to histogram
		val =  c* (a - (b * d))	
		histo[int(angle)] = val
	for i in range (0, 359):
		histo[i] = (histo[i] + histo[i+1])/2
	
	#identify valleys in histogram
	valleys = []
	start = -1
	end = -1
	for i in range (0, 360):
		#degrees with values less than 5 are considered valleys
		if(histo[i] < 5 and start == -1):
			start = i	
		elif(histo[i] < 5 and start > -1):
			end = i
		elif(histo[i] > 5 and end != -1):
			if(end - start > 10): #if a valley has a width > 10 degrees it is accepted
				valleys.append((start+end)/2)
			start = -1
			end = -1
		if(i == 359):
			end = i
			if(start > 0 and end - start > 10):
				valleys.append((start+end)/2)
			start = -1
			end = -1
		
	#calculate the goal heading
	headingDiff = robot.findDeltaHeadingTo(GOAL)
	if (headingDiff < 0):
		headingDiff = 360 - headingDiff
	rHeading = robot.getTh()
	if (rHeading < 0):
		rHeading = 360 + rHeading 
	goalHeading = headingDiff + rHeading
	
	#check if goal heading is free
	start = goalHeading - 10
	end = goalHeading + 10
	free = 0
	if (start < 0):
		start = 0
	if(end > 360):
		end = 360
	for i in range (int(start), int(end)):
		if(histo[i] < 5):
			free = free + 1
	if(free > 5 or not valleys):
		robot.setHeading(goalHeading)
	else: #if goal heading is blocked, set heading to closest valley
		closestValley = valleys[0]
		for i in valleys:
			if(difference(closestValley, goalHeading) > difference(i, goalHeading)):
				closestValley = i
		robot.setHeading(closestValley)
		

GOALX = 2500
GOALY = -7000
GOAL = ArPose(GOALX, GOALY)
while(robot.findDistanceTo(GOAL) > 100):
	robot.setVel(50)
	VFH(GOALX, GOALY, grid, GRID_SIZE)
	ArUtil_sleep(50)
robot.setVel(0)
print "GOAL REACHED!!"
