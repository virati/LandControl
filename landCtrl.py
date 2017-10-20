#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 20 15:29:38 2017

@author: virati
Main Class for Landscape Controller
"""

import numpy as np
import scipy.signal as sig
import matplotlib.pyplot as plt
from matplotlib import cm
from mpl_toolkits.mplot3d import Axes3D

from time import sleep

class LandCtrl:
    Z = []
    X = []
    Y = []
    dim = 0
    fig = []
    ax = []
    
    curr_state = []
    
    def __init__(self,curr_state=(0,0)):
        print('Controller Initialized')
        #dimensionality of the grid
        self.dim = 2
        self.Xl = np.linspace(0,1,1000)
        self.Yl = np.linspace(0,1,1000)
        
        self.curr_state = curr_state
        
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        pass
    
    def set_landscape(self,core='gaussian',param = [(0.5,0.5),(0.1,0.1)]):
        if core == 'gaussian':
            X,Y = np.meshgrid(self.Xl,self.Yl)
            xc = param[0][0]
            yc = param[0][1]
            
            xv = param[1][0]
            yv = param[1][1]
            
            #make the potential
            self.Z = -0.5 * np.exp(-(X - xc)**2 / (2 * xv **2) - (Y - yc)**2 / (2 * yv ** 2))
            self.X = X
            self.Y = Y
    def plot_landscape(self):
        surf = self.ax.plot_surface(self.X, self.Y, self.Z, cmap=cm.coolwarm,
                       linewidth=0, antialiased=False)
        
        #self.ax.plot3d(0,0,0)
        plt.draw()
        
    def set_target(self,coord):
        self.curr_target = coord
        
    def get_target(self):
        return self.curr_target
    
    def set_rotation(self,theta):
        #set the rotation matrix/transform from the input frame to the intrinsic class frame
        self.Trot = np.array([[np.cos(theta), -np.sin(theta)],[np.sin(theta),np.cos(theta)]])
        
    def rotate_vect(self,coord):
        #this class has a fixed reference frame where (0,0) is the origin. (1,1) is the other end, everything is normalized to be between
        #this function rotates
        
        #take the input argument coordinate/vector and transform it into the class's frame
        return np.dot(self.Trot,coord)
    
    def run(self):
        #this function loops through and moves the state closer to the target location
        while np.linalg.norm(self.curr_state - self.get_target()) > 0.001:
            print(self.curr_state)
            displ_vect = self.get_target() - self.curr_state

            #actuation
            
            self.curr_state += 0.01 * displ_vect/np.linalg.norm(displ_vect)
            sleep(0.1)
            self.plot_landscape()
        print('Reached Target')
if __name__ == '__main__':
    plt.ion()
    
    plt.show()


    test = LandCtrl()
    test.set_landscape()
    target = np.array([1,1])
    test.set_target(coord=target)
    test.set_rotation(theta=0)
    
    
    test.plot_landscape()
    
    test.run()
    
    