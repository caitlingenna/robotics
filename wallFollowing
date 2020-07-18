# VEX EDR Python-Project
import sys
import vex

#region config
ultrasonic_2 = vex.UltrasonicSensor(2, vex.UNIT_CM)
output_9     = vex.DigitalOutput(9)
input_10     = vex.DigitalInput(10)
input_11     = vex.DigitalInput(11)
motor_2      = vex.Motor(2)
motor_3      = vex.Motor(3)
motor_4      = vex.Motor(4)
motor_5      = vex.Motor(5)
#endregion config

ultrasonic_2.set_unit_cm()

def goForward(speed):
    motor_2.run(speed)
    motor_3.run(speed)
    motor_4.run(speed)
    motor_5.run(speed)
def turnRight(speed):
    motor_2.run(-speed)
    motor_3.run(-speed)
    motor_4.run(speed)
    motor_5.run(speed)
def turnLeft(speed):
    motor_2.run(speed)
    motor_3.run(speed)
    motor_4.run(-speed)
    motor_5.run(-speed)
def stop():
    motor_2.run(0)
    motor_3.run(0)
    motor_4.run(0)
    motor_5.run(0)


def findWall():
    distance = ultrasonic_2.distance()
    while(distance > 90):
        turnLeft(15) #turn left until a close wall is detected
        distance = ultrasonic_2.distance()
    stop()
    while(distance > 15):
        goForward(25) #go forward until within a foot of wall
        distance = ultrasonic_2.distance()
    stop()
def followWall():
    findWall()
    #turn away from wall and move forward
    turnRight(15)
    sys.sleep(2)
    goForward(25)
    sys.sleep(2)
    stop()
  
while(1):
    followWall()
