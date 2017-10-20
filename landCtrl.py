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

class LandCtrl:
    Z = []
    X = []
    Y = []
    dim = 0
    
    def __init__(self):
        print('Controller Initialized')
        #dimensionality of the grid
        self.dim = 2
        self.X = np.linspace(0,1,1000)
        self.Y = np.linspace(0,1,1000)
        pass
    
    def set_landscape(self,class='gaussian'):
        
    
    def plot_landscape(self):
        
    def set_target(self,coord):
        self.curr_target = coord
        
