#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 19 22:44:31 2019

@author: jason
"""

from marvelmind import MarvelmindHedge
from time import sleep
from sklearn.preprocessing import normalize
import sys
import numpy as np
import math


hedge12 = MarvelmindHedge(adr=12, tty="/dev/ttyACM0", baud=115200, debug=False) # create MarvelmindHedge thread
hedge12.start() # start thread

hedge22 = MarvelmindHedge(adr=22, tty="/dev/ttyACM1", baud=115200, debug=False) # create MarvelmindHedge thread
hedge22.start() # start thread
TimeStamp1 = 0
TimeStamp2 = 0
hedge1_pos = np.array([12, 0 , 0 , 0, 0],dtype='float32')
hedge2_pos = np.array([22, 0 , 0 , 0, 0],dtype='float32')
while True:
    try:
#        sleep(0.129)
#        for _ in range (2):
#            
#            
##            print (hedge.position()) # get last position and print
#            hedge12_position = np.asarray(hedge12.position(),dtype='float32')
#            if hedge12.position()[0] == 12 and TimeStamp1 == hedge12_position[5]:
#                break            
##            hedge12.print_position()
#            if hedge12.position()[0] == 12 and TimeStamp1 < hedge12_position[5]:
#                hedge1_pos = np.append(hedge12_position[0:4],hedge12_position[5]/10000)
#                TimeStamp1 = hedge12_position[5]
#                
#            if hedge12.position()[0] == 22 and TimeStamp1 == hedge12_position[5]:
#                break
#            if hedge12.position()[0] == 22 and TimeStamp2 < hedge12_position[5] :
#                hedge2_pos = np.append(hedge12_position[0:4],hedge12_position[5]/10000)
#                TimeStamp2 = hedge12_position[5]
##            print('pos 12:', hedge12_position)
#        
##            hedge_22_position = np.asarray(hedge_22.position(),dtype='float32')
##            print('pos 22:', hedge_22_position)
###            hedge_12_position = np.array((12, 2,-20,0),dtype='float32')
###            hedge_22_position = np.array((22, 2,2,0),dtype='float32')
#        hedge_position = np.vstack((hedge1_pos,hedge2_pos))
#        print (hedge_position)

##            col = len(hedge_position)
##            row = np.shape(hedge_position)
##            print(row)
#            left_pos = hedge_position[0,1:4]
#            right_pos= hedge_position[1,1:4]
#            zero_dot = (right_pos-left_pos)/2+left_pos
#            x_dot = normalize((right_pos-zero_dot).reshape(-1,1),axis=0)
#            rot_mat = np.array([[0,-math.sin(math.pi/2),0],
#                              [math.sin(math.pi/2),0,0],
#                              [0,0,1]],)
#            y_dot = (np.matmul(rot_mat,x_dot).T).flatten()
#            print('y_dot:', y_dot)
#            hedge.print_position()
        print(hedge22.valuesImuData[-1])
        print(hedge12.valuesImuData[-1])
#        [0.766, -4.003, -1.288, -0.01, 0.2231, -0.9729, 0.0596, 0.028, -0.019, 0.045, 0.479, -0.305, -19.719, 0, 1829225]
#        print(hedge12.valuesImuRawData[-1])
#            if (hedge.distancesUpdated):
#                hedge.print_distances()
    except KeyboardInterrupt:
#        hedge12.stop()  # stop and close serial port
        hedge22.stop()
        sys.exit()




