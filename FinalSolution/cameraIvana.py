#!/usr/bin/env python3
import sys
from robot_library.robot import *
import cv2
import numpy as np
import math
import hamming
import time

#initialization of robots' values
WHEEL_RADIUS = 0.09
robot = Robot()
laser = robot.getLaser()

#information of minimum distance tumba
minTumbaDistanceFromWall = 10000
minTumbaColor = "noColor"
minTumbaDistanceFromCorner = 10000

X = -1
Y = -1


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
    def forward(self, length):
        global WHEEL_RADIUS

        #length of a half of a sector
        semisector = length
        #aim direction to regulator
        current_dir = robot.getDirection()

        dist_left = (semisector / WHEEL_RADIUS)

        #initial value of encoders
        initial_enc = (robot.getEncoders().get("right"))

        #cycle for moving in semisector length
        while dist_left > 0.005:
            enc = (robot.getEncoders().get("right"))
            dist_left = initial_enc + (semisector / WHEEL_RADIUS) - enc

            error = current_dir - robot.getDirection()
            control = error * self.P + (self.lastError - error) * self.D
            robot.setVelosities(0.3, control)
            self.lastError = error
            time.sleep(0.001)

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
    #right = 1 - right wall move, right = 0 - left wall move
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

        if(right == 1):
            checkIndex = 113
        else:
            checkIndex = 480


        laser = laser.get('values')[40:len(laser.get('values'))-40]
        sumForward=1000
        while sumForward>0.26 and laser[checkIndex]<0.7:
            enc = abs(robot.getEncoders().get("left"))
            laser = robot.getLaser()
            laser = laser.get('values')[40:len(laser.get('values')) - 40]
            sumLeft = 0
            sumRight = 0
            for z in range(23,33):
                sumRight += laser[z]
            for z in range(492, 502):
                sumLeft += laser[z]
            sumRight2 = 0
            for z in range(93,103):
                sumRight2 += laser[z]

            sumForward = 1000
            for z in range(math.ceil(len(laser)/2)-50,math.ceil(len(laser)/2)+50):
                sumForward = min(sumForward,laser[z])

            if(right == 1):
                up = sumRight2
            else:
                up = sumLeft
            errorEnc = 3.3 - up

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
                if math.ceil((sumRight/10 + 0.31)/0.6)*0.6 < minTumbaDistanceFromWall:
                    minTumbaDistanceFromWall = math.ceil((sumRight/10 + 0.31)/0.6)*0.6
                    minTumbaDistanceFromCorner = enc - initial_enc
                    self.turn(1)
                    time.sleep(0.01)
                    minTumbaColor=colorDecoder()
                    self.turn(-1)
                    time.sleep(0.01)

            #then we move to the some barrier(wall or tumba) we should decrease our speed
            error = current_dir - robot.getDirection()
            control = (errorEnc * self.P)
            if sumForward>0.5:
                robot.setVelosities(0.7, control)
            else:
                robot.setVelosities(0.3, control)
            self.lastError = error
            time.sleep(0.001)

        #return of checking wall in the front
        robot.setVelosities(0,0)
        if not (sumForward>0.25):
            return 1
        else:
            return 0


    # function turns robot at right(1) or left(-1)
    def turn(self, ty):

        global robot

        add_deg_rad = 1.65
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
            time.sleep(0.001)

        robot.setVelosities(0, 0)


#initializating of object of class movement with P and D coefficients
mov = Movement(0.5, -1)


#move around yellow tumba to find ARTag
def moveAroundYellow():
    returnInfo = 1
    print("found")
    robot.sleep(10)
    returnMove = 0
    laser = robot.getLaser()
    laser = laser.get('values')[40:len(laser.get('values')) - 40]
    #move by left-handed regulator to find ARTag
    while(returnInfo == 1 and returnMove == 0):
        mov.turn(1)

        robot.setVelosities(0, 0)
        time.sleep(1)
        laser = robot.getLaser()
        laser = laser.get('values')[40:len(laser.get('values')) - 40]
        while laser[500] < 0.6:
            mov.forwardMin()
            laser = robot.getLaser()
            laser = laser.get('values')[40:len(laser.get('values')) - 40]

        robot.setVelosities(0,0)
        time.sleep(0.5)
        mov.forward(0.50)
        robot.setVelosities(0,0)
        time.sleep(0.5)
        #robot.sleep(0.1)
        laser = robot.getLaser()
        laser = laser.get('values')[40:len(laser.get('values')) - 40]
        sumLeft = 0
        for z in range(492, 502):
            sumLeft += laser[z]
        if (sumLeft > 3.5):
            mov.turn(-1)
            robot.setVelosities(0, 0)
            time.sleep(0.5)
            while (laser[580] < 0.6):
                mov.forwardMin()
                laser = robot.getLaser()
                laser = laser.get('values')[40:len(laser.get('values')) - 40]

            robot.setVelosities(0, 0)
            robot.sleep(0.5)
            mov.forward(0.55)
            robot.setVelosities(0, 0)
            robot.sleep(0.5)
            mov.turn(-1)
            robot.sleep(0.1)
            returnInfo = cameraDecode()
        else:
            returnMove = 1


    #move by right-handed regulator to find ARTag
    if returnMove == 1:
        returnInfo = 1
        while (returnInfo == 1):
            mov.turn(1)
            mov.turn(1)

            robot.setVelosities(0, 0)
            time.sleep(1)
            laser = robot.getLaser()
            laser = laser.get('values')[40:len(laser.get('values')) - 40]
            while laser[120] < 0.6:
                mov.forwardMin()
                laser = robot.getLaser()
                laser = laser.get('values')[40:len(laser.get('values')) - 40]

            robot.setVelosities(0, 0)
            time.sleep(0.5)
            mov.forward(0.50)
            robot.setVelosities(0, 0)
            time.sleep(0.5)
            # robot.sleep(0.1)
            laser = robot.getLaser()
            laser = laser.get('values')[40:len(laser.get('values')) - 40]
            sumRight = 0
            for z in range(103, 113):
                sumRight += laser[z]
            if (sumRight > 3.5):
                mov.turn(1)
                robot.setVelosities(0, 0)
                time.sleep(0.5)
                while (laser[120] < 0.6):
                    mov.forwardMin()
                    laser = robot.getLaser()
                    laser = laser.get('values')[40:len(laser.get('values')) - 40]

                robot.setVelosities(0, 0)
                robot.sleep(0.5)
                mov.forward(0.55)
                robot.setVelosities(0, 0)
                robot.sleep(0.5)
                mov.turn(1)
                robot.sleep(0.1)
                returnInfo = cameraDecode()
            else:
                returnMove = 1

def colorDecoder():
    # retreive the frame from robot's camera
    frame = robot.getImage()
    # Translate the frame into HSV colorspace
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
    # Get the pixel in the middle of the frame
    pix = hsv[360][640]
    # Compare the HUE with all the four colors
    redp = min((abs(pix[0] - 0)), (abs(pix[0] - 179)))
    greenp = (abs(pix[0] - 60))
    bluep = (abs(pix[0] - 120))
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


#function for detecting color in the picture
def cameraDecode():
    global X
    global Y
    robot.sleep(0.1)
    # retreive the frame from robot's camera
    frame = robot.getImage()
    # Translate the frame into HSV colorspace
    hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

    # define range of color in HSV
    lower = np.array([0, 127, 0])
    upper = np.array([255, 255, 255])

    # Threshold the HSV image to get only specific colors
    mask = cv2.inRange(hsv, lower, upper)
    hsv[mask > 0] = (0, 0, 255)


    lower = np.array([0, 0, 10])
    upper = np.array([255, 255, 255])

    # Threshold the HSV image to get only specific colors
    mask = cv2.inRange(hsv, lower, upper)
    mask=mask[:600][:]

    top=10000
    bottom=0
    left=10000
    right=0
    for i in range(len(mask)):
        for j in range(len(mask[1])):
            if(mask[i][j]<100):
                if j<left:
                    left=j
                if j>right:
                    right=j
                if i<top:
                    top=i
                if i>bottom:
                    bottom=i

    if(top != 10000 or bottom != 0):
        #print(top, bottom, left, right)

        # Extract Region of Interest from original
        artagart = mask[top:bottom, left:right]

        w=abs(right-left)
        h=abs(bottom-top)
        semistepw = w / 12
        semisteph = h / 12
        artag=[[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]

        for i in range(0,4):
            for j in range(0,4):
                if artagart[math.floor(i*semisteph*2+3*semisteph)][math.floor(j*semistepw*2+3*semistepw)]>0:
                    artag[i][j]=1
            #print(artagrot[i])
        rots=0
        while artag[3][3]!=1:
            rots+=1
            artag=list(zip(*artag[::-1]))
            if(rots>4):
                #print("ARTAG ERROR")
                break

        #for i in range(0,4):
        #    print(artag[i])
        show=artagart
        artagLine = np.array([0,1,0,1,1,0,1,1,0,1,0,0])

        index = 0
        for z in range(0,4):
            for z2 in range(0,4):
                if not ((z == 0 or z == 3) and (z2 == 0 or z2 ==3)):
                    artagLine[index] = artag[z][z2]
                    index+=1

        #applying hamming code
        h = hamming.Hamming(12)
        detectedData = h.decode_block(np.array(artagLine))


        X = detectedData[0] * 1 + detectedData[1] * 2 + detectedData[2] * 4
        Y = detectedData[3] * 1 + detectedData[4] * 2 + detectedData[5] * 4


        # outputs
        print("found marker")
        print("binary " + str(artagLine))
        print("coordinates " + str(X) + " " + str(Y))
        robot.sleep(10)

        return 2

    else:
        return 1


#main function
if __name__ == "__main__":

    #checking in which orientation we are
    #to turn our robot to the right rotation
    laser = robot.getLaser()
    laser = laser.get('values')[40:len(laser.get('values')) - 40]

    returnInfo = cameraDecode()

    #checking in which rotation we have yellow tumba

    if(returnInfo == 1):
        color = colorDecoder()
        if(color == "yellow"):
            moveAroundYellow()
        else:
            mov.turn(1)
            returnInfo = cameraDecode()
            if(returnInfo == 1):
                color = colorDecoder()
                if(color == "yellow"):
                    moveAroundYellow()
                else:
                    mov.turn(1)
                    returnInfo = cameraDecode()
                    if(returnInfo == 1):
                        color = colorDecoder()
                        if(color == "yellow"):
                            moveAroundYellow()
                        else:
                            mov.turn(1)
                            returnInfo = cameraDecode()
                            if(returnInfo == 1):
                                color = colorDecoder()
                                if(color == "yellow"):
                                    moveAroundYellow()


    exit(1)
