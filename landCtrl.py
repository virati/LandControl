#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Oct 20 15:29:38 2017

@author: virati
Main Class for Landscape Controller
"""

import pdb
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
    
    def __init__(self,start_state=(0.001,0.001)):
        print('Controller Initialized')
        #dimensionality of the grid
        self.dim = 2
        self.Xl = np.linspace(0,1,1000)
        self.Yl = np.linspace(0,1,1000)
        
        self.curr_state = start_state
        
        self.fig = plt.figure()
        self.ax = self.fig.gca(projection='3d')
        pass
    
    def set_landscape(self,core='gaussian'):
        if core == 'gaussian':
            X,Y = np.meshgrid(self.Xl,self.Yl)
            xc = self.curr_target[0]
            yc = self.curr_target[1]
            
            xv = 0.1
            yv = 0.1
            
            #make the potential
            self.Z = -0.5 * np.exp(-(X - xc)**2 / (2 * xv **2) - (Y - yc)**2 / (2 * yv ** 2))
            self.X = X
            self.Y = Y
            #put the absolute edges at very high potential
            wall = 2 * np.ones_like(self.Z[0,:])
            self.Z[1,:] = wall
            self.Z[-2,:] = wall
            self.Z[:,1] = wall
            self.Z[:,-2] = wall
            
    def plot_landscape(self):
        #surf = self.ax.plot_surface(self.X, self.Y, self.Z, cmap=cm.coolwarm, linewidth=0, antialiased=False)
        plt.cla()
        self.ax.plot_wireframe(self.X,self.Y,self.Z,rstride=120,cstride=120)
        #self.ax.plot3d(0,0,0)
        self.ax.scatter(self.curr_state[0],self.curr_state[1],0)
        vg = 0.01
        #print(self.sdot)
        self.ax.quiver(self.curr_state[0],self.curr_state[1],0,vg*self.sdot[0],vg*self.sdot[1],0,arrow_length_ratio=0.1)
        self.ax.set_zlim(-1,1)
        plt.draw()
        plt.title(self.curr_state)
        plt.pause(0.0001)
        
    def plot_lnd_deriv(self):
        grad = np.array(np.gradient(self.Z))
        
        fig = plt.figure()
        ax = fig.gca(projection='3d')

        pdb.set_trace()
        plt.subplot(211)
        ax.plot_wireframe(self.X,self.Y,grad[0],rstride=120,cstride=120)
        plt.subplot(212)
        ax.plot_wireframe(self.X,self.Y,grad[1],rstride=120,cstride=120)


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

    def lin_ctrl(self):
        godir = self.get_target() - self.curr_state
        return (godir) / np.linalg.norm(godir)    

    def gen_grad(self):
        self.gradgrid = np.array(np.gradient(self.Z))


    def land_ctrl(self):
        #where is the current position?
        curloc = self.curr_state
        #what's the gradient of the lanscape at this point?
        #find where the CURRENT index is
        xidx = np.argmin(np.abs(self.X - curloc[0])[0,:])
        yidx = np.argmin(np.abs(self.Y - curloc[1])[:,0])
    
        #now, go to that point in the x and y
        xbase = self.gradgrid[0][xidx,yidx]
        ybase = self.gradgrid[1][xidx,yidx]
        
        #and check its neighbors
        xneigh = self.gradgrid[0][xidx-1:xidx+1,yidx]
        yneigh = self.gradgrid[1][xidx,yidx-1:yidx+1]

        #negate the first and then find the largest abs for the direction to go
        xneigh[0] = -xneigh[0]
        yneigh[0] = -yneigh[0]
        
        xgo_dir = 2*(np.argmin(xneigh) - 0.5)
        ygo_dir = 2*(np.argmin(yneigh) - 0.5)
        #so now we have a direction to go for x and y
    
    
        #return a vector with the above that is unit
        retvect = np.array([xgo_dir,ygo_dir])
    
        #let's find the magnitude of the potential itself
        #gfact = np.abs(10*self.Z[xidx,yidx])
    
        #and we'll now incorporate this, by making the output magnitude inversely related to the potential magnitude
        
        #gfact = np.linalg.norm(retvect)
    
        gfact = 1
        self.sdot = retvect / gfact
        return retvect / gfact

    def run(self):
        #this function loops through and moves the state closer to the target location
        while np.linalg.norm(self.curr_state - self.get_target()) > 0.01:
            #print(self.curr_state)
            
            ctrl = 'land'
            if ctrl == 'lin':
                act = self.lin_ctrl()

            elif ctrl == 'land':
                _ = self.land_ctrl()
                act = self.sdot
                
            #actuation
            g1 = 0.01

            #compress the actuation
            actn = g1 * np.tanh(act - 0.5)
            
            print(act)

            self.curr_state += actn
            sleep(0.001)
            self.plot_landscape()
        print('Reached Target')
if __name__ == '__main__':
    plt.ion()
    
    test = LandCtrl(start_state = (0.1,0.9))
    
    #Where is the target?
    target = np.array([0.5,0.5])
    test.set_target(coord=target)
    
    #decide and implement the landscape potential function
    test.set_landscape()
    #Set where the target is
    
    #Set the rotation between the initial/robot frame and the global frame
    test.set_rotation(theta=0)
    
    #Plot the landscape of the potential
    #test.plot_landscape()
    
    test.gen_grad()
    #test.plot_lnd_deriv()
    
    test.run()
