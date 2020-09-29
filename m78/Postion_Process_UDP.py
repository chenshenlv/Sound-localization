#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 14 14:00:24 2019

@author: jason
"""

from UDPBeacon import BeaconUDP
from Kalman import Kalman
import time
from help import help_fun
import numpy as np
import collections
import struct
import sys
import threading

class Postion_Process(threading.Thread):
    def __init__(self, udp_left = None,udp_right = None):
        self.udp_left = udp_left
        self.udp_right = udp_right
#        self.TimeStamp1 = 0
#        self.TimeStamp2 = 0
        self.hp_fun = help_fun()
#        self.hedge1_pos = np.array([0 , 0 , 0],dtype='float32')
#        self.hedge2_pos = np.array([0 , 0 , 0],dtype='float32')
        self.terminaterequire = False
        self.k_filter1=Kalman()
        self.k_filter2=Kalman()
        maxvaluescount = 3
        self.hedge1_pos = collections.deque([[0]*3]*maxvaluescount, maxlen=maxvaluescount)
        self.hedge2_pos = collections.deque([[0]*3]*maxvaluescount, maxlen=maxvaluescount)
        self.pos_buffer = collections.deque([[0]*6]*maxvaluescount, maxlen=maxvaluescount)
        self.x_in=np.zeros((maxvaluescount,6),dtype='float32')
        self.x_out=np.zeros((1,6),dtype='float32')
        threading.Thread.__init__(self)
        
        
    def pos_proc(self):
        return self.pos_buffer[-1]
    def stop(self):
        self.terminaterequire = True         
    def run(self):
        
        while (not self.terminaterequire):
            try:
                hedge1_pos_raw=self.udp_left.request_position()
                hedge1_filtered_x, hedge1_filtered_y=self.k_filter1.K_filter(hedge1_pos_raw[0],hedge1_pos_raw[1])
                hedge1_pos_filtered=np.array([hedge1_filtered_x,hedge1_filtered_y,0])
                
                hedge2_pos_raw=self.udp_right.request_position()
                hedge2_filtered_x, hedge2_filtered_y=self.k_filter2.K_filter(hedge2_pos_raw[0],hedge2_pos_raw[1])
                hedge2_pos_filtered=np.array([hedge2_filtered_x,hedge2_filtered_y,0])
                
                self.hedge1_pos.append(hedge1_pos_filtered)
                self.hedge2_pos.append(hedge2_pos_filtered)
                self.x_in=np.delete(self.x_in,0,0)
#                print('H2:', self.hedge2_pos)
#                self.x_in=np.vstack((self.x_in,[self.i+0.2,self.i-0.2,0,self.i-0.11,self.i-0.11,self.i-0.11]))
#                self.x_in=np.vstack((self.x_in,np.hstack((self.hedge1_pos, self.hedge2_pos))))
                self.x_in=np.vstack((self.x_in,self.hp_fun.paired_position(self.hedge1_pos[-1], self.hedge2_pos[-1])))
                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
#                print ('hedge12_position',self.x_out)
                self.pos_buffer.append(self.x_out)
                time.sleep(0.01)
            except KeyboardInterrupt:
                self.stop()
                sys.exit()
        
            except struct.error:
                self.x_in=np.vstack((self.x_in,[1.1,-2.1,0,0,-3.1,0]))

                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
                self.pos_buffer.append(self.x_out)