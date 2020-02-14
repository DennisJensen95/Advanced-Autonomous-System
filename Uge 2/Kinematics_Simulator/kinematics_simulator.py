import numpy as np
import time
import matplotlib.pyplot as plt

class Kinematics():

    def __init__(self):
        self.cur_pose = None
        self.last_pose = None
        self.robot_parameters = None
        self.wheelspeed = None
        self.pose_vec = []

    def XiDifferentialDrive(self, theta, r_r, r_l, vel_r, vel_l, w, ts):
        # Theis' udregning differential drive
        vel_r = self.vel_r / self.robot_parameters[1]
        vel_l = self.vel_l / self.robot_parameters[2]
        self.wheelspeed = (vel_r, vel_l)
        rolling_constraint_front = (np.cos(theta) * (r_l * vel_l + r_r * vel_r)) / 2 * ts
        sliding_constraint_front = -(np.sin(theta) * (r_l * vel_l + r_r * vel_r)) / 2 * ts
        sliding_constraint_back = (r_l * vel_l - r_r * vel_r) / (2 * w) * ts

        return np.array([rolling_constraint_front, sliding_constraint_front, sliding_constraint_back])

    def KinUpdate(self, pose=None, robotpar=None, ts=None, wheelspeed=None):
        pose = self.cur_pose if pose == None else pose
        self.cur_pose = pose

        robotpar = self.robot_parameters if robotpar == None else robotpar
        self.robot_parameters = robotpar

        ts = self.ts if ts == None else ts
        self.ts = ts

        wheelspeed = self.wheelspeed if wheelspeed == None else wheelspeed
        self.wheelspeed = wheelspeed

        cur_x, cur_y, cur_theta = pose
        w, r_r, r_l = robotpar
        w_vel_r, w_vel_l = wheelspeed
        self.last_pose = pose
        pose = np.array(pose) + self.XiDifferentialDrive(cur_theta, r_r, r_l, w_vel_r, w_vel_l, w, ts)
        self.cur_pose = pose
        return pose

    def GoForward(self, dist, speed):
        self.vel_r = speed
        self.vel_l = speed
        self.start_pose = self.cur_pose
        while True:
            dist_gone = np.linalg.norm(np.array(self.start_pose[:2]) - np.array(self.cur_pose[:2]))
            if dist_gone >= dist:
                break
            else:
                pose = self.KinUpdate()
                self.pose_vec.append(pose)

    def PrintDrive(self):
        plt.figure()
        for point in self.pose_vec:
            plt.plot(point[0], point[1], 'bo')

        plt.grid()
        plt.show()

    def Turn(self, radians, speed):
        if radians > 0:
            self.vel_r = speed
            self.vel_l = -speed
        else:
            self.vel_r = -speed
            self.vel_l = speed

        start_theta = self.cur_pose[2]
        while True:
            Turned_angle_rad = abs(self.cur_pose[2] - start_theta)
            if Turned_angle_rad >= abs(radians):
                break
            else:
                pose = self.KinUpdate()
                self.pose_vec.append(pose)

    def Move2Pose(self):
        """
        Moving to pose
        :return:
        """


kin = Kinematics()
pose = (0, 0, 0)
robotpar = (0.26, 0.035, 0.035)
ts = 0.01
wheelspeed = (2.86, 2.86)

pose_vec = []

kin.cur_pose = pose
kin.robot_parameters = robotpar
kin.ts = ts
kin.wheelspeed = wheelspeed

state = 'star'

# Square
if state == 'square':
    kin.GoForward(1, 2)
    kin.Turn(np.pi / 2, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 2, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 2, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 2, 1)

# Hexagon
if state == 'hexagon':
    kin.GoForward(1, 2)
    kin.Turn(np.pi / 3, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 3, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 3, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 3, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 3, 1)
    kin.GoForward(1, 3)
    kin.Turn(np.pi / 3, 1)

# Start
if state == 'star':
    kin.GoForward(1,1)
    kin.Turn(108*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(-36*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(108*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(-36*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(108*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(-36*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(108*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(-36*np.pi/180,0.1)
    kin.GoForward(1,1)
    kin.Turn(108*np.pi/180,0.1)
    kin.GoForward(1,1)

kin.PrintDrive()

