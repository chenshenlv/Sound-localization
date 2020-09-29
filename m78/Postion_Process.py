#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Sep 16 23:56:23 2019

@author: jason
"""



from marvelmind import MarvelmindHedge
import time
from help import help_fun
import numpy as np
import collections
import struct
import sys
import threading

class Postion_Process(threading.Thread):
    def __init__(self, serial=None, x_in=None):
        self.serial=serial
        self.TimeStamp1 = 0
        self.TimeStamp2 = 0
        self.x_in=x_in
        self.x_out=np.zeros((1,6),dtype='float32')
        self.hp_fun = help_fun()
#        self.hedge1_pos = np.array([0 , 0 , 0],dtype='float32')
#        self.hedge2_pos = np.array([0 , 0 , 0],dtype='float32')
        self.terminaterequire = False
        self.i= 0
        maxvaluescount = 3
        self.hedge1_pos = collections.deque([[0]*3]*maxvaluescount, maxlen=2)
        self.hedge2_pos = collections.deque([[0]*3]*maxvaluescount, maxlen=2)
        self.pos_buffer = collections.deque([[0]*6]*maxvaluescount, maxlen=maxvaluescount)
        threading.Thread.__init__(self)
        
        
    def pos_proc(self):
        return self.pos_buffer[-1]
    def stop(self):
        self.terminaterequire = True         
    def run(self):

        while (not self.terminaterequire):
            try:
                hedge12_position = np.asarray(self.serial.position(),dtype='float32')
            
                if hedge12_position[0] == 12 and self.TimeStamp1 < hedge12_position[5]:
#                    self.hedge1_pos = hedge12_position[1:4]
                    self.hedge1_pos.append(hedge12_position[1:4])

                    self.TimeStamp1 = hedge12_position[5]
                    

                if hedge12_position[0] == 22 and self.TimeStamp2 < hedge12_position[5] :
#                    self.hedge2_pos = hedge12_position[1:4]
                    self.hedge2_pos.append(hedge12_position[1:4])
                    self.TimeStamp2 = hedge12_position[5]
                self.x_in=np.delete(self.x_in,0,0)
#                print('H2:', self.hedge2_pos)
#                self.x_in=np.vstack((self.x_in,[self.i+0.2,self.i-0.2,0,self.i-0.11,self.i-0.11,self.i-0.11]))
#                self.x_in=np.vstack((self.x_in,np.hstack((self.hedge1_pos, self.hedge2_pos))))
                self.x_in=np.vstack((self.x_in,self.hp_fun.paired_position(self.hedge1_pos[-1], self.hedge2_pos[-1])))
                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
#                print ('hedge12_position',self.x_out)
                self.pos_buffer.append(self.x_out)
                self.i+=0.1
                time.sleep(0.001)
            except KeyboardInterrupt:
                self.stop()
                sys.exit()
                break
                exit
        
            except struct.error:
                self.x_in=np.vstack((self.x_in,[1.1,-2.1,0,0,-3.1,0]))

                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
                self.pos_buffer.append(self.x_out)
      