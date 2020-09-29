#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Oct 23 13:25:20 2019

@author: jason
"""

from UDPBeacon import BeaconUDP
from Kalman import Kalman
import numpy as np
import time
import matplotlib.pyplot as plt
import sys
ip = '192.168.1.6'
port = 18888
beacon_id12 = 12
pos_arry=np.array([0,0])
pos_filtered_arry=np.array([0,0])
k_filter=Kalman()
udp12 = BeaconUDP(ip, port, beacon_id12)
udp12.start()

while True:
    try:
        a12=udp12.request_position()
        x_coord,y_coord=k_filter.K_filter(a12[0],a12[1])
        a12_filtered=np.array([x_coord,y_coord])
        pos_arry=np.vstack((pos_arry,a12[0:2]))
        pos_filtered_arry=np.vstack((pos_filtered_arry,a12_filtered[0:2]))
        time.sleep(0.3)
    except KeyboardInterrupt:
        np.save('pos_raw.npy',pos_arry.T)
        np.save('pos_filtered.npy',pos_filtered_arry.T)
        plt.plot(pos_arry.T[0,0:len(pos_arry)],pos_arry.T[1,0:len(pos_arry)],'r',pos_filtered_arry.T[0,0:len(pos_filtered_arry)],pos_filtered_arry.T[1,0:len(pos_filtered_arry)],'b')
        plt.legend(('raw path','filtered path'),loc='lower center')
        udp12.close()
        exit
        break
    
#a=np.array([0,0])
#b=np.array([0,1])
#a=np.vstack((a,b))
pos_arry.T[0,0:len(pos_arry)]