#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jun 24 02:01:52 2019

@author: jason
"""

from scipy.io import loadmat
from help import objects, help_fun
from marvelmind import MarvelmindHedge
from time import sleep
import threading 
import matplotlib.pyplot as plt  
import numpy as np
import sys
from multiprocessing import Process, Queue
from help import objects, help_fun

def smooth(q,x_in,hedge):   
    while q.full()!=True:
        try:
            x_in=np.delete(x_in,0,0)
            x_in=np.vstack((x_in,hedge.valuesImuData[-1][0:7]))
            x_out[0,:]=np.sum(x_in,axis=0)/30
            q.put_nowait(x_out[0,:])
            return x_in
        except KeyboardInterrupt:
            print('\nInterrupted by user')   


            
if __name__=='__main__':
    q=Queue()
    buffer_size=3
    q = Queue(maxsize=buffer_size)
    hedge = MarvelmindHedge(adr=None, tty="/dev/ttyACM0", baud=9600, debug=False) # create MarvelmindHedge thread
    hedge.start() # start thread
    
    x_in=np.zeros((30,7),dtype='float32')
    
    x_out=np.zeros((1,7),dtype='float32')
    
    for _ in range(buffer_size):
        for i in range (len(x_in)):
            sleep(0.001)
            x_in[i,:]=hedge.valuesImuData[-1][0:7]
            x_out[0,:] = np.sum(x_in,axis=0)/30
        q.put_nowait(x_out[0,:])
        
        obj_body=np.zeros((1000,3),dtype='float32')
        azimuths_body=np.zeros((1000,1),dtype='float32')

        hp_fun=help_fun()
        
        for i in range(len(azimuths_body)):
            p = Process(target=smooth, args=(q,x_in,hedge,))
            p.start()
            output=q.get_nowait()
            hedge_iner_posi=output[0:3]
            hedge_quan=output[3:7]
            obj_body[i,:]=hp_fun.quaternionRotate(hedge_quan,np.array([1,0,0])) #object's position in hedge body system
            azimuths_body[i,:]=hp_fun.calculateAzimuths(obj_body[i,:])
        p.join()
        plt.plot(azimuths_body)  
        

#    while True:
#        try:
#            sleep(0.001)
#            # print (hedge.position()) # get last position and print
#            for i in range (30):
#                
#        except KeyboardInterrupt:
#            hedge.stop()  # stop and close serial port
#            sys.exit()\

