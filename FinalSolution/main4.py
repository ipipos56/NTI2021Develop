#!/usr/bin/env python3
from robot_library.robot import *
import hamming
import math
import time as tm
import cv2
import numpy as np

#initialization of robots' values
WHEEL_RADIUS = 0.09
robot = Robot()
laser = robot.getLaser()

#information of minimum distance tumba
minTumbaDistanceFromWall = 10000
minTumbaColor = "noColor"
minTumbaDistanceFromCorner = 10000

locX = 0
locY = 0
rotation = 3

X = -1
Y = -1

class RobotConst:  # константы
    cell_size = 0.304
    exact_cell_size = 0.3
    wheel_d = 0.17999999999999999
    track = 0.29999999999999999
    cpr = 2 * math.pi

    forward_laser_delta = 0.154  # на столько м вынесен дальномер вперед центра оси
    min_lasers_on_edge = 15  # столько лазеров должно попасть на ребро, чтобы считать его препятствием
    laser_node_collision_dist = 0.10  # расстояние от точки на ребре до узла должно быть не меньше этого (в клетках)
    laser_collision_dist = 0.05  # точка на ребре, если расхождение ее координаты и настоящей координаты ребра не больше этого (в клетках)
    lasers_on_one_node = 4  # столько лазеров проверяет один узел на пустоту; будет взято x2+1
    min_length_delta = 0.2  # каждый лазер должен проходить дальше пустого узла хотя бы на столько м

    v = 0.35
    vslow = 0.04
    acceleration = 0.2  # points per second
    pid_window_size = 10
    decel_start_offset = 1 # расстояние до начала замедления
    rotate_decel_start_offset = 29000  # расстояние до начала замедления
    maze_size = 15

    hamming_length = 12
    directions = [(-1, 0), (0, 1), (1, 0), (0, -1)]
    colors = {
        2: 'green',
        4: 'blue',
        0: 'red',
        1: 'yellow'
    }

class Data:  # одометрия
    cur_angle = 0
    pid_error_deque = [0.0 for i in range(RobotConst.pid_window_size)]
    pid_deque_index = -1
    field = np.full((RobotConst.maze_size * 2 + 1, RobotConst.maze_size * 2 + 1), -1)
    field_cells = [[-1 for _ in range((RobotConst.maze_size + 1) * 2 + 1)]
                   for _ in range((RobotConst.maze_size + 1) * 2 + 1)]
    robot_pos = (RobotConst.maze_size, RobotConst.maze_size)
    borders = [1, 1, field.shape[0] - 2, field.shape[1] - 2]
    enc = {"left": 0, "right": 0}
    sensors = {'time_stamp': 0, 'angle': 0.0, 'angle_increment': 0.0, 'values': tuple([0.0 for _ in range(683)])}
    encL = 0
    encR = 0
    min_robot_pos = (RobotConst.maze_size, RobotConst.maze_size)
    max_robot_pos = (RobotConst.maze_size, RobotConst.maze_size)
    finish = False

# Shortcut methods
robot = Robot()
eL = lambda: Data.enc['right']
eR = lambda: Data.enc['left']
sF = lambda: min(Data.sensors["values"][(683 // 2) - 40:(683 // 2) + 41])
sR = lambda: Data.sensors['values'][31]
sR90 = lambda: Data.sensors['values'][84]
sL = lambda: Data.sensors['values'][-32]
sL90 = lambda: Data.sensors['values'][-85]
node_value = lambda cell_dist: (cell_dist % 1 <= RobotConst.laser_node_collision_dist or
                                cell_dist % 1 >= (1 - RobotConst.laser_node_collision_dist))
edge_value = lambda cell_dist: (cell_dist % 1 <= RobotConst.laser_collision_dist or
                                cell_dist % 1 >= (1 - RobotConst.laser_collision_dist))
pi = math.pi
inf = float('inf')
sleep = lambda x: tm.sleep(x / 1000)
eps = 0.00000001

sign = lambda x: -1 if x < 0 else 1
time = lambda: tm.time() * 1000
getYaw = lambda: math.degrees(robot.getDirection()) * -1000


def motors(vL=None, vR=None):
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    vL = RobotConst.v if vL is None else vL
    vR = vL if vR is None else vR

    delta = vL - vR
    linear = vR + delta / 2
    angular = (2 * pi) / (pi * RobotConst.track / (delta / 2)) if abs(delta) > eps else 0
    robot.setVelosities(linear, angular)

def coordinateUpdate():
    global locY
    global locX
    if rotation == 0:
        locY+=0.5
    elif rotation == 2:
        locY-=0.5
    elif rotation == 1:
        locX+=0.5
    elif rotation == 3:
        locX-=0.5


def forward_gyro(cm=float(RobotConst.cell_size), additional_corrections=lambda: 0.0):
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    gyro_kp = 0.000005
    path = (cm / (pi * RobotConst.wheel_d)) * RobotConst.cpr
    Data.encL += path
    Data.encR += path

    sgn = sign(cm)
    flag = 0 if eL() < Data.encL else 1
    while [eL(), Data.encL][flag] < [Data.encL, eL()][flag]:
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        extra_correction = additional_corrections()
        if abs(Data.encL - eL()) < RobotConst.decel_start_offset:
            power = RobotConst.vslow
        else:
            power = RobotConst.v
        error = Data.cur_angle - getYaw()
        if error > 180000:
            error -= 360000
        if error < -180000:
            error += 360000
        correction = error * gyro_kp
        motors(max(0, min(power + correction * sgn + extra_correction, RobotConst.v * 2)) * sgn,
               max(0, min(power - correction * sgn - extra_correction, RobotConst.v * 2)) * sgn)
        if Data.finish:
            exit(1)
        sleep(10)
    motors(0)
    coordinateUpdate()
    robot.sleep(0.1)

def forward_gyro_while(check_func, dir=1, power=RobotConst.v):
    """
    Едет по гироскому пока check_func возвращает True
    :param check_func: функция, по которой надо останавливать цикл (возвращает bool)
    :param dir: 1 - вперёд, -1 - назад
    :param power: скорость
    :return:
    """
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    gyro_kp = 0.00002  # П коэффиент для езды по гироскопу
    sgn = sign(dir)
    while check_func():
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        error = Data.cur_angle - getYaw()
        if error > 180000:
            error -= 360000
        if error < -180000:
            error += 360000
        correction = error * gyro_kp
        motors(max(0, min(power + correction * sgn, 100)) * sgn, max(0, min(power - correction * sgn, 100)) * sgn)
        if Data.finish:
            exit(1)
        sleep(1)
    Data.encL = eL()
    Data.encR = eR()
    motors(0)


def proximity_corrector():
    """
    Corrects the robot position according to the walls position
    :return:
    """

    if sF() <= 0.142:
        forward_gyro_while(lambda: sF() < 0.146, -1, RobotConst.vslow)
        sleep(250)
    if 0.152 <= sF() <= 0.3:
        forward_gyro_while(lambda: sF() > 0.148, 1, RobotConst.vslow)
        sleep(250)

#class movement for all robots' moves
class Movement:
    P=0
    D=0
    lastError = 0

    #basic constructor for initializing P and D coefficients for PD regulator
    def __init__(self, P, D):
        self.P = P
        self.D = D

    #wall-laser movement until the wall in front appear
    #right = 1 - right wall move, right = 0 - left wall move
    def moveAlongWall(self,right):

        global WHEEL_RADIUS
        global minTumbaDistanceFromWall
        global minTumbaDistanceFromCorner
        global minTumbaColor

        noiseCounter = 0


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
            forward_gyro()
            time.sleep(0.001)

        #return of checking wall in the front
        robot.setVelosities(0,0)
        if not (sumForward>0.25):
            return 1
        else:
            return 0

    def rotate_gyro_absolute(self,angle):
        """
        Rotates robot to a given value (NOTE: accepts values from -180 to 180 degrees!!!)
        """
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        if abs(angle) < 200:
            angle *= 1000
        delta = angle - Data.cur_angle
        if delta > 180000:
            delta -= 360000
        if delta < -180000:
            delta += 360000
        cm = RobotConst.track * pi * (delta / 360000)
        Data.encL += (cm / (pi * RobotConst.wheel_d)) * RobotConst.cpr
        Data.encR -= (cm / (pi * RobotConst.wheel_d)) * RobotConst.cpr
        Data.cur_angle = angle
        sgn = sign(angle - getYaw())
        if abs(angle - getYaw()) >= 180000:
            sgn *= -1
        if Data.finish:
            exit(1)
        while abs(angle - getYaw()) > 3000 and (angle > -180000 + 3000 or abs(-angle - getYaw()) > 3000):
            Data.sensors = robot.getLaser()
            Data.enc = robot.getEncoders()
            if abs(angle - getYaw()) < RobotConst.rotate_decel_start_offset or \
                    (angle <= -180000 + RobotConst.rotate_decel_start_offset and
                     abs(-angle - getYaw()) < RobotConst.rotate_decel_start_offset):
                motors(RobotConst.vslow * 1.3 * sgn, RobotConst.vslow * 1.3 * -sgn)
            else:
                motors((RobotConst.v / 3 * 2) * sgn, (RobotConst.v / 3 * 2) * -sgn)
            sleep(1)
            if Data.finish:
                exit(1)
        motors(0)
        if abs(eL() - Data.encL) > 2 * math.pi or abs(eL() - Data.encL) > 2 * math.pi:
            Data.encL = eL()
            Data.encR = eR()

    def rotate_gyro_relative(self,angle):
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        angle *= 1000
        self.rotate_gyro_absolute(((Data.cur_angle + angle + 180000) % 360000) - 180000)

    # function turns robot at right(1) or left(-1)
    def turn(self, dir):
        global rotation
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        proximity_corrector()
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()

        if sign(dir) > 0 and sL() < 0.437 and abs(0.317 - sL()) > 0.05:
            self.rotate_gyro_relative(-90)
            proximity_corrector()
            self.rotate_gyro_relative(90)
        elif sign(dir) < 0 and sR() < 0.437 and abs(0.317 - sR()) > 0.05:
            self.rotate_gyro_relative(90)
            proximity_corrector()
            self.rotate_gyro_relative(-90)
        self.rotate_gyro_relative(sign(dir) * 90)
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        proximity_corrector()
        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        if dir == 1:
            rotation+=1
            rotation%=4
        else:
            rotation+=3
            rotation%=4
        robot.sleep(0.1)


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

    Data.cur_angle = getYaw()
    wallBorder = 3.7

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

    start = False

    #we should go through all corners to the start
    while locX != 0 or locY != 0 or not start:

        Data.sensors = robot.getLaser()
        Data.enc = robot.getEncoders()
        if abs(locX - 0) + abs(locY-0) > 3:
            start = True
        sumForward = 0
        sumLeft = 0
        sumRight = 0
        for z in range(100, 110):
            sumRight += laser[z]
        for z in range(492, 502):
            sumLeft += laser[z]
        for z in range(math.ceil(len(laser) / 2)-5, math.ceil(len(laser) / 2)+5):
            sumForward += laser[z]
        if sumLeft < wallBorder:
            mov.turn(-1)
            forward_gyro()
        elif sumForward < wallBorder:
            forward_gyro()
        else:
            mov.turn(1)



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
