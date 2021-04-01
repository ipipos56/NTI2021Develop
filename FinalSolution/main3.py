import math
import collections
from threading import Thread
from robot_library.robot import *
import time as tm
from scripts.artag_detector import *


DEBUG = True


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


# -1 - неизвестная клетка
#  0 - пустота
#  1 - препятствие
#  2 - граница поля


# Shortcut methods
robot = Robot()
detector = ARTagDetector(debug=DEBUG)
hamming = Hamming(RobotConst.hamming_length, debug=DEBUG)
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
sleep = lambda x: tm.sleep(x / 1000)
inf = float('inf')
eps = 0.00000001

sign = lambda x: -1 if x < 0 else 1
time = lambda: tm.time() * 1000
getYaw = lambda: math.degrees(robot.getDirection()) * -1000
log = lambda *args, sep=' ', end='\n', file=None: print(*args, sep=sep, end=end, file=file) if DEBUG else None

def motors(vL=None, vR=None):
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    vL = RobotConst.v if vL is None else vL
    vR = vL if vR is None else vR

    delta = vL - vR
    linear = vR + delta / 2
    angular = (2 * pi) / (pi * RobotConst.track / (delta / 2)) if abs(delta) > eps else 0
    robot.setVelosities(linear, angular)


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
        log("CORRECT TOO CLOSE FRONT")
        forward_gyro_while(lambda: sF() < 0.146, -1, RobotConst.vslow)
        sleep(250)
    if 0.152 <= sF() <= 0.3:
        log("CORRECT TOO FAR FRONT")
        forward_gyro_while(lambda: sF() > 0.148, 1, RobotConst.vslow)
        sleep(250)

def rotate_gyro_absolute(angle):
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
    log('Rotate', angle, 'done!')


def rotate_gyro_relative(angle):
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    angle *= 1000
    rotate_gyro_absolute(((Data.cur_angle + angle + 180000) % 360000) - 180000)
    log('Rotate', angle, '(relative) done!')


def precise_90_degree_turn(dir):
    """
    Turns robot in a maze by 90 degrees precisely
    :param dir: 1 - clockwise, -1 - counterclockwise
    :return:
    """
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    proximity_corrector()
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()

    if sign(dir) > 0 and sL() < 0.437 and abs(0.317 - sL()) > 0.05:
        rotate_gyro_relative(-90)
        proximity_corrector()
        rotate_gyro_relative(90)
    elif sign(dir) < 0 and sR() < 0.437 and abs(0.317 - sR()) > 0.05:
        rotate_gyro_relative(90)
        proximity_corrector()
        rotate_gyro_relative(-90)
    rotate_gyro_relative(sign(dir) * 90)
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    proximity_corrector()
    Data.sensors = robot.getLaser()
    Data.enc = robot.getEncoders()
    robot.sleep(0.1)

if __name__ == '__main__':
    Data.cur_angle = getYaw()
    for i in range(1,7):
        forward_gyro()

    precise_90_degree_turn(1)
    forward_gyro()
    forward_gyro()
    forward_gyro()
    forward_gyro()
    precise_90_degree_turn(-1)
    forward_gyro()
    forward_gyro()
    forward_gyro()
    forward_gyro()
    forward_gyro()
    forward_gyro()
    precise_90_degree_turn(-1)
    forward_gyro()
    forward_gyro()
    precise_90_degree_turn(1)
    forward_gyro()
    forward_gyro()
    precise_90_degree_turn(1)
    forward_gyro()
    forward_gyro()
    forward_gyro()
    precise_90_degree_turn(1)
    forward_gyro()
    forward_gyro()
    exit(1)