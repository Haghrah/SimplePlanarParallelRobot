#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Tue Apr 10 10:45:16 2018

@author: arslan
"""

from numpy import sin, cos, array, deg2rad, arctan2
from scipy.optimize import newton_krylov
import matplotlib.pyplot as plt


class Robot(object):
    
    def __init__(self, o1, o2, l1, l2, l3, l4, NK_iter=None):
        self.o1 = array(o1)
        self.o2 = array(o2)
        self.l1 = l1
        self.l2 = l2
        self.l3 = l3
        self.l4 = l4
        self.theta1 = 0
        self.theta2 = 0
        self.c1 = array([0, 0])  # link o1-c1 length l1
        self.c2 = array([0, 0])  # link o2-c2 length l2
        self.c3 = array([0, 0])  #end effector, c1-c3 length l3, c2-c3 length l4
        self.NK_iter = NK_iter
    
    def __repr__(self):
        o = ""
        o += "O1: " + str(self.o1[0]) + ", " + str(self.o1[1])
        o += "\nO2: " + str(self.o2[0]) + ", " + str(self.o2[1])
        o += "\ntheta1: " + str(self.theta1)
        return o
    
    def crossing_circles_direct(self, x):
        '''For finding c3!'''
        eq1 = (x[0] - self.c1[0]) ** 2 + (x[1] - self.c1[1]) ** 2 - self.l3 ** 2
        eq2 = (x[0] - self.c2[0]) ** 2 + (x[1] - self.c2[1]) ** 2 - self.l4 ** 2
        return array([eq1, eq2])
    
    def crossing_circles_inverse_c1(self, x):
        '''For finding c1!'''
        eq1 = (x[0] - self.c3[0]) ** 2 + (x[1] - self.c3[1]) ** 2 - self.l3 ** 2
        eq2 = (x[0] - self.o1[0]) ** 2 + (x[1] - self.o1[1]) ** 2 - self.l1 ** 2
        return array([eq1, eq2])
    
    def crossing_circles_inverse_c2(self, x):
        '''For finding c3!'''
        eq1 = (x[0] - self.c3[0]) ** 2 + (x[1] - self.c3[1]) ** 2 - self.l4 ** 2
        eq2 = (x[0] - self.o2[0]) ** 2 + (x[1] - self.o2[1]) ** 2 - self.l2 ** 2
        return array([eq1, eq2])
    
    def direct_kinematics(self, theta1, theta2, eq_x0 = [0, 0]):
        self.theta1 = theta1
        self.theta2 = theta2
        x1 = self.o1[0] + self.l1 * cos(self.theta1)
        y1 = self.o1[1] + self.l1 * sin(self.theta1)
        self.c1 = array([x1, y1])        
        x2 = self.o2[0] - self.l2 * cos(self.theta2)
        y2 = self.o2[1] + self.l2 * sin(self.theta2)
        self.c2 = array([x2, y2])
        if self.NK_iter is None:
            sol = newton_krylov(self.crossing_circles_direct, eq_x0)
        else:
            sol = newton_krylov(self.crossing_circles_direct, eq_x0, iter=self.NK_iter)
        self.c3[0] = sol[0]
        self.c3[1] = sol[1]
        return self.c3
    
    def inverse_kinematics(self, c3, eq1_x0 = [0, 0], eq2_x0 = [0, 0]):
        self.c3 = array(c3)
        if self.NK_iter is None:
            sol1 = newton_krylov(self.crossing_circles_inverse_c1, eq1_x0)
        else:
            sol1 = newton_krylov(self.crossing_circles_inverse_c1, eq1_x0, iter=self.NK_iter)
        self.c1[0] = sol1[0]
        self.c1[1] = sol1[1]
        if self.NK_iter is None:
            sol2 = newton_krylov(self.crossing_circles_inverse_c2, eq2_x0)
        else:
            sol2 = newton_krylov(self.crossing_circles_inverse_c2, eq2_x0, iter=self.NK_iter)
        self.c2[0] = sol2[0]
        self.c2[1] = sol2[1]
        self.theta1 = arctan2(self.c1[1] - self.o1[1], self.c1[0] - self.o1[0])
        self.theta2 = arctan2(self.c2[1] - self.o2[1], self.c2[0] - self.o2[0])
        return self.theta1, self.theta2
    
    def plot(self):
        plt.figure()
        plt.plot([self.o1[0], self.c1[0]], [self.o1[1], self.c1[1]])
        plt.plot([self.c1[0], self.c3[0]], [self.c1[1], self.c3[1]])
        plt.plot([self.o2[0], self.c2[0]], [self.o2[1], self.c2[1]])
        plt.plot([self.c2[0], self.c3[0]], [self.c2[1], self.c3[1]])
        plt.grid(True)
        
        xmin = min(self.o1[0], self.o2[0], self.c1[0], self.c2[0], self.c3[0])
        xmax = max(self.o1[0], self.o2[0], self.c1[0], self.c2[0], self.c3[0])
        ymin = min(self.o1[1], self.o2[1], self.c1[1], self.c2[1], self.c3[1])
        ymax = max(self.o1[1], self.o2[1], self.c1[1], self.c2[1], self.c3[1])
        
        plt.xlim(xmin-5, xmax+5)
        plt.ylim(ymin-5, ymax+5)
        plt.show()
        
if __name__ == "__main__":
    myRobot = Robot((-10, 0), (10, 0), 7, 7, 7, 7)
    t1 = deg2rad(45)
    t2 = deg2rad(45)
    myRobot.direct_kinematics(t1, t2)
    myRobot.plot()
    x = 0
    y = 2
    myRobot.inverse_kinematics((x, y))
    myRobot.plot()





















