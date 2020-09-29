#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Oct 20 23:19:48 2019

@author: jason
"""
import numpy as np
class Kalman:
    def __init__(self, ):
        self.P=10 #predict covariance
        self.Q=3 #world uncertainty covariance
        self.R=30 #measurement uncertainty
        self.x_pre=0
        self.y_pre=0
        
    def K_filter(self,x_meature, y_meature):
        self.P=self.P+self.Q
#        print('P:',self.P)
        self.kalman_gain=self.P/(self.P+self.R)
#        print('kalman_gain:',self.kalman_gain)
        self.x_pre=self.x_pre+self.kalman_gain*(x_meature-self.x_pre)
        self.y_pre=self.y_pre+self.kalman_gain*(y_meature-self.y_pre)
#        print('x,y:',np.array([self.x_pre,self.y_pre]))
        self.P=self.P*(1-self.kalman_gain)
#        print('P:',self.P)
        return self.x_pre, self.y_pre