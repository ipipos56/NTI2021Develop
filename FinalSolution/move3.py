#!/usr/bin/env python3

import sys
from robot_library.robot import *
import cv2
import numpy as np
import math

#initialization of robots' values
WHEEL_RADIUS = 0.09
robot = Robot()
laser = robot.getLaser()

#information of minimum distance tumba
minTumbaDistanceFromWall = 10000
minTumbaColor = "noColor"
minTumbaDistanceFromCorner = 10000


#class movement for all robots' moves
class Movement:
    P=0
    D=0
    lastError = 0

    #basic constructor for initializing P and D coefficients for PD regulator
    def __init__(self, P, D):
        self.P = P
        self.D = D

    #movement forward by gyroscope
    def forward(self):
        global WHEEL_RADIUS

        #length of a half of a sector
        semisector = 0.35
        #aim direction to regulator
        current_dir = robot.getDirection()

        dist_left = (semisector / WHEEL_RADIUS)

        #initial value of encoders
        initial_enc = abs(robot.getEncoders().get("left"))

        #cycle for moving in semisector length
        while dist_left > 0.005:
            enc = abs(robot.getEncoders().get("left"))
            dist_left = initial_enc + (semisector / WHEEL_RADIUS) - enc

            error = current_dir - robot.getDirection()
            control = error * self.P + (self.lastError - error) * self.D
            robot.setVelosities(0.3, control)
            self.lastError = error
            robot.sleep(0.001)

        robot.setVelosities(0, 0)

    #gyroscope moving for one iteration
    def forwardMin(self):
        global WHEEL_RADIUS

        semisector = 0
        current_dir = robot.getDirection()
        error = current_dir - robot.getDirection()
        control = error * self.P
        robot.setVelosities(0.3, control)
        self.lastError = error


    #wall-laser movement until the wall in front appear
    def moveAlongWall(self,right):

        global WHEEL_RADIUS
        global minTumbaDistanceFromWall
        global minTumbaDistanceFromCorner
        global minTumbaColor

        semisector = 0.2085
        noiseCounter = 0
        current_dir = robot.getDirection()

        #this variable save last distance to the wall from a last corner
        #because robot should not check one tumba twice
        lastDistanceFromCornerToTumba = 0

        initial_enc = abs(robot.getEncoders().get("left"))

        laser = robot.getLaser()

        laser = laser.get('values')[40:len(laser.get('values'))-40]
        sumForward=1000
        while sumForward>0.25 and laser[480]<0.7:
            enc = abs(robot.getEncoders().get("left"))
            laser = robot.getLaser()
            laser = laser.get('values')[40:len(laser.get('values')) - 40]
            sumLeft = 0
            sumRight = 0
            for z in range(23,33):
                sumRight += laser[z]
            for z in range(492, 502):
                sumLeft += laser[z]

            sumForward = 1000
            for z in range(math.ceil(len(laser)/2)-50,math.ceil(len(laser)/2)+50):
                sumForward = min(sumForward,laser[z])
            print(sumForward)
            errorEnc = 3.3 - sumLeft

            #if in the right laser values we see tumba then we increase counter
            #because it can be not a tumba, just noise
            if(sumRight < 20):
                noiseCounter+=1
            else:
                noiseCounter=0


            #if we know that in the right we see tumba and we see this tumba in a first time, then we can check distance to it
            if noiseCounter >=4 and (laser[math.ceil(len(laser)/2)]>0.4) and ((enc-initial_enc)-lastDistanceFromCornerToTumba > 0.6/WHEEL_RADIUS or lastDistanceFromCornerToTumba == 0):
                noiseCounter = 0
                lastDistanceFromCornerToTumba = enc - initial_enc
                #if distance less then minimum tumba then we can update minimum tumba and save color for it
                if sumRight < minTumbaDistanceFromWall:
                    minTumbaDistanceFromWall = sumRight
                    minTumbaDistanceFromCorner = enc - initial_enc
                    self.turn(1)
                    robot.sleep(0.01)
                    minTumbaColor=colDec()
                    self.turn(-1)
                    robot.sleep(0.01)

            #then we move to the some barrier(wall or tumba) we should decrease our speed
            error = current_dir - robot.getDirection()
            control = (errorEnc * self.P)
            if sumForward>0.5:
                robot.setVelosities(0.7, control)
            else:
                robot.setVelosities(0.3, control)
            self.lastError = error
            robot.sleep(0.001)

        #return of checking wall in the front
        robot.setVelosities(0,0)
        if not (sumForward>0.25):
            return 1
        else:
            return 0


    # function turns robot at right(1) or left(-1)

    def turn(self, ty):

        global robot

        add_deg_rad = 1.6
        # defining some constants
        MAX_TURN_SPEED = 0.1
        P_COEF = 0.4

        current_dir = robot.getDirection()
        # calculate target direction of robot after turn
        target_dir = current_dir + add_deg_rad*ty

        # calculate error of rotation (nobody knows how it works, but it does)
        e = (target_dir - current_dir + np.pi * 5) % (np.pi * 2) - (np.pi)

        # accepting threshold after turn is 0.01

        while abs(e - np.sign(e) * np.pi) > 0.005:
            current_dir = robot.getDirection()
            e = (target_dir - current_dir + np.pi * 5) % (np.pi * 2) - (np.pi)
            turn_speed = -(e - np.sign(e) * np.pi) * 1 + np.sign(e) * 0.1

            # limit our speed with MAX_TURN_SPEED bound
            turn_speed = np.sign(turn_speed) * np.maximum(np.abs(turn_speed), MAX_TURN_SPEED)
            # equivalent to bottom line
            # turn_speed = (turn_speed if turn_speed > -MAX_TURN_SPEED else -MAX_TURN_SPEED) if turn_speed < MAX_TURN_SPEED else MAX_TURN_SPEED

            robot.setVelosities(0, turn_speed)

            # some delay for don't overload computation
            robot.sleep(0.001)

        robot.setVelosities(0, 0)

#function for detecting color in the picture
def colDec():
    # retreive the frame from robot's camera
    frame = robot.getImage()
    # Translate the frame into HSV colorspace
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    #Get the pixel in the middle of the frame
    pix=hsv[360][640]
    #Compare the HUE with all the four colors
    redp=min((abs(pix[0]-0)),(abs(pix[0]-179)))
    greenp=(abs(pix[0]-60))
    bluep=(abs(pix[0]-120))
    yellowp=(abs(pix[0]-30))
    #Return the right color
    if(redp<greenp and redp<bluep and redp<yellowp):
        return "red"
    elif(greenp<redp and greenp<bluep and greenp<yellowp):
        return "green"
    elif(bluep<redp and bluep<greenp and bluep<yellowp):
        return "blue"
    elif(yellowp<redp and yellowp<bluep and yellowp<greenp):
        return "yellow"


#main function
if __name__ == "__main__":

    #initializating of object of class movement with P and D coefficients
    mov = Movement(0.5,-1)

    #checking in which orientation we are
    #to turn our robot to the right rotation
    laser = robot.getLaser()
    laser = laser.get('values')[40:len(laser.get('values')) - 40]

    sumLeft = 0
    sumRight = 0
    for z in range(100, 110):
        sumRight += laser[z]
    for z in range(492, 502):
        sumLeft += laser[z]
    if(sumLeft<5 and laser[math.ceil(len(laser)/2)]<0.5):
        mov.turn(1)
    elif(sumLeft>5 and laser[math.ceil(len(laser)/2)]>0.5):
        mov.turn(-1)
    elif (sumLeft > 5 and laser[math.ceil(len(laser) / 2)] < 0.5):
        mov.turn(-1)
        mov.turn(-1)

    corners = 0
    moveAroundTumba = 0

    #we should go through all corners to the start
    while corners < 4:

        #move near to the wall
        stopState=mov.moveAlongWall(0)

        laser = robot.getLaser()
        laser = laser.get('values')[40:len(laser.get('values')) - 40]

        #if we see wall in the front, then it will be corner or tumba
        if stopState==1:
            laser = robot.getLaser()
            laser = laser.get('values')[40:len(laser.get('values')) - 40]

            #if we see blocks in right and left, then we should turn around
            #and go around 2 tumba which are blocking our robot
            if (laser[0] < 0.6 and laser[602] < 0.6):
                mov.turn(1)
                mov.turn(1)
            else:
                mov.turn(1)
                corners+=1
                moveAroundTumba = 0
            #if it is a tumba and we rotated to the first tumbas' edge, then
            #we should go until we see air in the "left" lasers
        elif laser[math.ceil(len(laser)/2)]>0.6:
            while laser[580]<0.6:
                mov.forwardMin()
                laser = robot.getLaser()
                laser = laser.get('values')[40:len(laser.get('values')) - 40]

            mov.forward()
            mov.turn(-1)
            moveAroundTumba+=1
            while laser[580]>0.6:
                mov.forwardMin()
                laser = robot.getLaser()
                laser = laser.get('values')[40:len(laser.get('values')) - 40]
            #moving in the last edge of a tumba
        if laser[math.ceil(len(laser) / 2)] < 0.6:
            initial_enc = abs(robot.getEncoders().get("left"))
            enc=initial_enc
            while laser[math.ceil(len(laser)/2)]>0.25:
                enc = abs(robot.getEncoders().get("left"))
                mov.forwardMin()
                laser = robot.getLaser()
                laser = laser.get('values')[40:len(laser.get('values')) - 40]
            #if we have at least 1 edge of tumba, then we should check it for a color
            #because it can be minimum distance from a wall tumba
            if(moveAroundTumba >=1):
                if(minTumbaDistanceFromWall > (enc-initial_enc) * WHEEL_RADIUS - 0.6):
                    minTumbaDistanceFromWall = (enc-initial_enc) * WHEEL_RADIUS - 0.6
                    mov.turn(-1)
                    corners-=2
                    minTumbaColor=colDec()
                    #print(minTumbaColor)
                    mov.turn(1)
            moveAroundTumba = 0
        robot.sleep(0.1)


    #outputs
    print("distance " + str(math.ceil(minTumbaDistanceFromWall)))
    print("color " + minTumbaColor)
    print("temp coordinates 0 0")
    if minTumbaColor == "blue":
        print("coordinates 0 7")
    elif minTumbaColor == "red":
        print("coordinates 7 7")
    elif minTumbaColor == "yellow":
        print("coordinates 7 0")
    elif minTumbaColor == "green":
        print("coordinates 0 0")
    exit(1)
