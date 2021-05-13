from connectsonar import *
from matplotlib import pyplot as plt

# Create a 100 by 100 list to function as an occupancy grid
grid = [0] * 100
for i in range(0, 100):
    grid[i] = [0] * 100

def readSonar():
# the range of accepted sonar values is -400 to 1600 to compensate the diamond shape, as part of the object has a negative coordinate position in reference to the robot's starting position (0,0)
    for s in range(0, robot.getNumSonar()):
   	 x = robot.getSonarReading(s).getX()
   	 y = robot.getSonarReading(s).getY()
   	 if (x <= 1600 and x > -400) and (y <= 1600 and y > -400):
   		 x = int((x + 400)/20) # 400 is added to make the range positive 0-2000. Then divided by 20 to fit into the 0-100 grid.
   		 y = int((y + 400)/20)
   		 grid[x][y] = grid[x][y] + 1    

# During testing the starting position of the robot was always the original (0,0) position as upon start of MobileSim, and the robot was reset to this position in between each trial and map.

h = 0 # used to keep track of robot heading
while h < 360: #while robot has not circumnavigated the object
    robot.move(45)
    while not robot.isMoveDone():
   	 readSonar()
    if robot.getSonarRange(0) < 900 or robot.getSonarRange(1) < 900 or robot.getSonarRange(2) < 900 or robot.getSonarRange(3) < 900:
    # if robot gets too close to object (as signalled by front or left sensors), turn away
   	 h = h - 3    	 
   	 robot.setHeading(h)
    if robot.getSonarRange(0) > 950 or robot.getSonarRange(15) > 950:
   # if robot gets too far from object, turn towards
   # degree of which to turn away is larger than that to turn towards, in case of overlapping signals
   	 h = h + 2   	 
   	 robot.setHeading(h)

# Display occupancy grid as an image
plt.imshow(grid)
axes = plt.gca()
axes.invert_yaxis() #to correct inverted y-axis
plt.show()
