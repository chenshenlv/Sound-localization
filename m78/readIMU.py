#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 19 23:33:40 2019

@author: jason
"""

from marvelmind import MarvelmindHedge
from time import sleep
import sys
import numpy as np
from help import help_fun
 
#class readIMU(MarvelmindHedge):
#    def __ini__(self, adr, tty, baud, debug):
#        self.adr=adr
#        self.tty=tty
#        self.baud=baud
#        self.debug=debug
#        

def hedge_position(self):
#    hp_fun=help_fun()
#    hedge.start() # start thread
    self.hedge=MarvelmindHedge(self.adr, self.tty, self.baud, self.debug) # create MarvelmindHedge thread
    while True:
        try:
            sleep(0.09)
            # print (hedge.position()) # get last position and print
            hedge_iner_posi=hedge.valuesImuData[-1][0:3] #hedge's position in inertial system
            hedge_quan=hedge.valuesImuData[-1][3:7]      #hedge's quatenion to the inertial system
#            print (hedge.valuesImuData[-1])
            print (hedge_iner_posi)
            print (hedge_quan)
#            obj_body=hp_fun.quaternionRotate(hedge_quan,obj_position) #object's position in hedge body system
#            azimuths=hp_fun.calculateAzimuths(obj_body) 
#            print (azimuths)
            return hedge_iner_posi,hedge_quan
            
        except KeyboardInterrupt:
            self.hedge.stop()  # stop and close serial port
            sys.exit()
#    return hedge_iner_posi,hedge_quan

