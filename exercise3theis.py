# -*- coding: utf-8 -*-
"""
Created on Tue Feb 11 15:06:10 2020

@author: Theis
"""

import numpy as np
import time
import matplotlib.pyplot as plt

class Kinematics():

    def __init__(self):
        self.pose = (0, 0, 0)
        self.robotpar = (0.26, 0.035, 0.035)
        self.ts = 0.01
        self.wheelspeed = (2.86, -2.86)
        self.distDriven = 0

        self.pose_vec = []
        self.pose_vec.append(self.pose)
        plt.figure()


    def kinupdate(self):
        pose = self.pose_vec[-1]
        th = pose[2]        
        poseUpdate = [0,0,0]
        
        for i in range(3):
            poseUpdate[i] += pose[i]
        
        poseUpdate[0] += (np.cos(th)*(self.robotpar[1]*self.wheelspeed[0]+self.robotpar[2]*self.wheelspeed[1]))/2*self.ts
        poseUpdate[1] += -(np.sin(th)*(self.robotpar[1]*self.wheelspeed[0]+self.robotpar[2]*self.wheelspeed[1]))/2*self.ts
        poseUpdate[2] += ((self.robotpar[1]*self.wheelspeed[0]-self.robotpar[2]*self.wheelspeed[1]))/2/self.robotpar[0]*self.ts
        
        self.pose_vec.append(poseUpdate)
        self.distDriven += np.linalg.norm(np.array(poseUpdate[:2])-np.array(pose[:2]))

        
        return poseUpdate
    
    def fwddrive(self, distance,velocity):
        distMarker = self.distDriven
        temp1 = velocity/self.robotpar[1]
        temp2 = velocity/self.robotpar[2]
        self.wheelspeed = (temp1, temp2)
        
        while abs(self.distDriven-distMarker) < distance:
            self.kinupdate()

        
    def turn(self, angle, speed):
        angleMarker = self.pose_vec[-1][2]
        
        if angle >= 0:
            temp1 = -speed/self.robotpar[1]
            temp2 = speed/self.robotpar[2]
        else:
            temp1 = speed/self.robotpar[1]
            temp2 = -speed/self.robotpar[2]
        self.wheelspeed = (temp1, temp2)
        
        angle_turned = 0
        while abs(angle_turned) < abs(angle):
            self.kinupdate()
            angle_turned = self.pose_vec[-1][2] - angleMarker


    def print_drive(self):
        plt.figure()
        for i in range(len(self.pose_vec)):
            if i%4:
                plt.plot(self.pose_vec[i][0], self.pose_vec[i][1], 'o', color='r')
        plt.gca().set_aspect('equal', adjustable='box')
        plt.grid()
        plt.show()


##square
robot1 = Kinematics()    

robot1.fwddrive(3,1)
robot1.turn(np.pi/2,0.1)
robot1.fwddrive(3,1)
robot1.turn(np.pi/2,0.1)
robot1.fwddrive(3,1)
robot1.turn(np.pi/2,0.1)
robot1.fwddrive(3,1)
robot1.turn(np.pi/2,0.1)
robot1.print_drive()


##hexagon
robot2 = Kinematics()    

robot2.fwddrive(1,1)
robot2.turn(60*np.pi/180,0.1)
robot2.fwddrive(1,1)
robot2.turn(60*np.pi/180,0.1)
robot2.fwddrive(1,1)
robot2.turn(60*np.pi/180,0.1)
robot2.fwddrive(1,1)
robot2.turn(60*np.pi/180,0.1)
robot2.fwddrive(1,1)
robot2.turn(60*np.pi/180,0.1)
robot2.fwddrive(1,1)
robot2.turn(60*np.pi/180,0.1)
robot2.fwddrive(1,1)
robot2.turn(60*np.pi/180,0.1)
robot2.print_drive()


##star
robot3 = Kinematics()

robot3.fwddrive(1,1)
robot3.turn(108*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(-36*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(108*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(-36*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(108*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(-36*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(108*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(-36*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.turn(108*np.pi/180,0.1)
robot3.fwddrive(1,1)
robot3.print_drive()