import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

class Kinematics():

    def __init__(self):
        self.cur_pose = None
        self.robot_parameters = None
        self.wheelspeed = [0, 0]
        self.pose_vec = []
        self.ts = None
        self.v = []
        self.w = []

        # FeedbackController
        self.k_values = [0.3, 0.8, -0.15]

    def PrintDrive(self):
        plt.figure()
        for point in self.pose_vec:
            plt.plot(point[0], point[1], 'bo')

        plt.xlim([-2, 2])
        plt.ylim([-2, 2])
        currentAxis = plt.gca()
        currentAxis.add_patch(Rectangle((self.pose_vec[-1][0] - 0.015, self.pose_vec[-1][1] - 0.015), 0.03, 0.03, facecolor="grey"))
        mag = np.sqrt(self.pose_vec[-1][0]**2 + self.pose_vec[-1][1]**2)
        x, y = mag * np.cos(self.pose_vec[-1][2]), mag * np.sin(self.pose_vec[-1][2])
        plt.quiver(self.pose_vec[-1][0], self.pose_vec[-1][1], x, y, scale=4)
        plt.grid()

    def PlotTheta(self):
        plt.figure()
        time = np.linspace(0, len(self.pose_vec)*self.ts, len(self.pose_vec))

        theta_vec = []
        for pose in self.pose_vec:
            theta_vec.append(pose[2])

        plt.plot(time, np.array(theta_vec)*180/np.pi, 'r-')

    def PlotVelocities(self):

        plt.figure()
        time = np.linspace(0, len(self.w) * self.ts, len(self.w))
        plt.plot(time, np.array(self.w))
        plt.title('Angular velocity')

        plt.figure()
        plt.plot(time, np.array(self.v))
        plt.title('Linear Velocity')

    def AngleWithinMinusPiToPi(self, angle):
        angle = np.arctan2(np.sin(angle), np.cos(angle))
        return angle

    def XiDifferentialDrive2(self):

        x = (np.cos(self.cur_pose[2]) * (self.robot_parameters[2] * self.wheelspeed[1] + self.robot_parameters[1] * self.wheelspeed[0])) / 2 * self.ts
        y = (np.sin(self.cur_pose[2]) * (self.robot_parameters[2] * self.wheelspeed[1] + self.robot_parameters[1] * self.wheelspeed[0])) / 2 * self.ts
        theta = (-self.robot_parameters[2] * self.wheelspeed[1] + self.robot_parameters[1] * self.wheelspeed[0]) / (self.robot_parameters[0]) * self.ts

        return np.array([x, y, theta])

    def XiDifferentialDrive(self):
        R = np.array([[np.cos(self.cur_pose[2]), np.sin(self.cur_pose[2]), 0],
                     [-np.sin(self.cur_pose[2]), np.cos(self.cur_pose[2]), 0],
                      [0, 0, 1]])
        A = np.array([[1, 0, self.robot_parameters[0]/2],
                      [1, 0, -self.robot_parameters[0]/2],
                      [0, 1, 0]])
        b = np.array([self.robot_parameters[1] * self.wheelspeed[0], self.robot_parameters[2] * self.wheelspeed[1], 0])

        pose = np.linalg.inv(R) @ np.linalg.inv(A) @ b * self.ts
        return pose

    def KinUpdate(self):
        self.cur_pose = self.cur_pose + self.XiDifferentialDrive()
        self.cur_pose[2] = self.AngleWithinMinusPiToPi(self.cur_pose[2])
        self.pose_vec.append(self.cur_pose)

    def Move2PoseController(self, inputPose):
        k = 0 # iterations
        print(f'Desired position: {inputPose}')
        self.targetPose = inputPose + self.cur_pose
        print(f'Target position: {self.targetPose}')

        theta_target = self.targetPose[2]
        self.Transform = np.array([[np.cos(theta_target), np.sin(theta_target), 0],
                                   [-np.sin(theta_target), np.cos(theta_target), 0],
                                   [0, 0, 1]])

        while True:
            xx = self.cur_pose - self.targetPose
            pose_global = self.Transform @ xx
            xT = pose_global[0]
            yT = pose_global[1]
            theta_T = pose_global[2]

            rho = np.sqrt(xT**2 + yT**2)
            alpha = -theta_T + np.arctan2(-yT, -xT)
            alpha = self.AngleWithinMinusPiToPi(alpha)
            beta = -theta_T - alpha
            beta = self.AngleWithinMinusPiToPi(beta)

            v = self.k_values[0] * rho
            omega = self.k_values[1] * alpha + self.k_values[2] * beta


            self.wheelspeed[0] = (2*v + omega * self.robot_parameters[0]) / (2 * self.robot_parameters[1])
            self.wheelspeed[1] = (2*v - omega * self.robot_parameters[0]) / (2 * self.robot_parameters[2])

            d_error = np.linalg.norm(self.cur_pose[:2] - self.targetPose[:2])
            a_error = np.pi - abs(abs(self.cur_pose[2] - self.targetPose[2]) - np.pi)

            if d_error < 0.01 and a_error < 0.02:
                break
            else:
                self.KinUpdate()

            if k > 5000:
                print('Maximum iterations reached')
                break



            k += 1

    def Turn(self, radians, speed):
        if radians > 0:
            self.wheelspeed[0] = speed
            self.wheelspeed[1] = -speed
        else:
            self.wheelspeed[0] = -speed
            self.wheelspeed[1] = speed

        start_theta = self.cur_pose[2]
        while True:
            Turned_angle_rad = abs(self.cur_pose[2] - start_theta)
            if Turned_angle_rad >= abs(radians):
                break
            else:
                self.KinUpdate()

    def GoForward(self, dist, speed):
        self.wheelspeed[0] = speed
        self.wheelspeed[1] = speed
        start_pose = self.cur_pose
        while True:
            dist_gone = np.linalg.norm(np.array(start_pose[:2]) - np.array(self.cur_pose[:2]))
            if dist_gone >= dist:
                break
            else:
                self.KinUpdate()

    def Move2Pose(self, inputPose, vel, turning_vel):
        """
        Moving to pose
        :return:
        """
        input_theta = inputPose[2]
        targetPose = inputPose - self.cur_pose

        rho = np.sqrt(targetPose[0] ** 2 + targetPose[1] ** 2)
        alpha = -targetPose[2] + np.arctan2(-targetPose[1], -targetPose[0])
        alpha = self.AngleWithinMinusPiToPi(alpha)

        self.Turn(alpha, turning_vel)
        self.GoForward(rho, vel)
        turn_back = (input_theta - alpha)
        self.Turn(turn_back, vel)


kin = Kinematics()
kin.cur_pose = np.array([0, 0, 0])
kin.robot_parameters = np.array([0.26, 0.035, 0.035])
kin.ts = 0.1

inputPose = np.array([0, 1, np.pi/2])

# kin.Move2Pose(inputPose, 1, 1)
kin.Move2PoseController(inputPose)

state = 'neither'

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
plt.show()