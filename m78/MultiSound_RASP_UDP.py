#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 14 17:31:44 2019

@author: jason
"""

from Postion_Process_UDP import Postion_Process
from Sound_UDP import Sound
from Client import client
from scipy.io import loadmat
from spatializer import spatialization
from readIMU import hedge_position 
from help import objects, help_fun
from UDPBeacon import BeaconUDP
import time
import math
import numpy as np
import collections
import struct
#import random
import sys
#import matplotlib.pyplot as plt
#import time as time
import sounddevice as sd
import soundfile as sf
import queue  # Python 3.x
import threading



sd.query_devices()
    
hp_fun=help_fun()
obj1 = objects(5.91,-3.5, 0, './SoundTrack/SoftBall_Charles_DeTore--4db.wav')
obj2 = objects(7.42, -3.5, 0, './SoundTrack/RiverStreamAdjusted--4db.wav')
obj3 = objects(10.81, -3.5, 0, './SoundTrack/mono2002-11-22t04--4db.wav')
obj4 = objects(19.81, -3.5, 0, './SoundTrack/SoftBall_Charles_DeTore--4db.wav')
obj5 = objects(21.69, -3.5, 0, './SoundTrack/RiverStreamAdjusted--4db.wav')
#obj6 = objects(-1.57, -5.01, 0, './SoundTrack/mono2002-11-22t04--4db.wav')
obj_position_list=[obj1.position, obj2.position, obj3.position, obj4.position, obj5.position]
objsound=[obj1.soundfile, obj2.soundfile, obj3.soundfile, obj4.soundfile, obj5.soundfile]
HRTF_data = loadmat('./HRTF/58_Propcessed_HRTF.mat')
azimuths =HRTF_data['azimuths'].flatten()
azimuths = azimuths.tolist()
previous_playlist = []

padding_len = len(np.squeeze(HRTF_data['hrir_l'][0, :]))-1 # 200*1

#event = threading.Event() 
#ip_add='192.168.1.15'  #ip address for GUI
#port = 8080
#socket = client()
#socket.connect(ip_add,port)

block_size = 8000 #frames size Sampling frequency in Hertz (= frames per second).
buffer_size = 6 #number of blocks used for buffering
    
ip = '192.168.1.6' #ip address for Marvelmind Modem
port = 18888       #port for Marvelmind UDP server
beacon_id_left = 21  #beacon id1
beacon_id_right = 22
udp_left = BeaconUDP(ip, port, beacon_id_left) # create Beacon UDP thread
udp_left.start() # start thread
udp_right = BeaconUDP(ip, port, beacon_id_right) # create Beacon UDP thread
udp_right.start() # start thread
time.sleep(1)
              
pos_get = Postion_Process(udp_left = udp_left, udp_right = udp_right) #create Beacon postion require thread
pos_get.start() #start thread
time.sleep(1)
for _ in range (10):
    x_out=pos_get.pos_proc()
    hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
    sorted_dis_index = hp_fun.distance(obj_position_list,hedge_iner_posi)[2]
print('sorted_dis_index initial',sorted_dis_index)    
for i in range(2):
    sound_thread = Sound(obj_position_list=obj_position_list[sorted_dis_index[i]],pos_get=pos_get,soundfile=objsound[sorted_dis_index[i]],HRTF_data=HRTF_data,
                                azimuths=azimuths,padding_len=padding_len,block_size=block_size,
                                buffer_size=buffer_size)
    sound_thread.name = str(sorted_dis_index[i])
    sound_thread.start()
    previous_playlist.append(sorted_dis_index[i])
time.sleep(0.5)    
distance=0
while True:
    try:

        x_out=pos_get.pos_proc()
        hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
#        hedge_iner_posi=np.array([0, distance, 0])
#        distance+=0.1
#        if distance > 5.0:
#            distance = 0.1
#        socket.send(str(hedge_iner_posi).encode('utf-8'))
        print('hedge_iner_posi',hedge_iner_posi)
#        hedge_quan=x_out[3:7]
#        hedge_quan=np.array([0.0122,0.089,0.202,0.0455])
        thread_switcher_array=hp_fun.distance(obj_position_list,hedge_iner_posi)[0]
#        print(thread_switcher_array)
        sorted_dis = hp_fun.distance(obj_position_list,hedge_iner_posi)[1]
        sorted_dis_index = hp_fun.distance(obj_position_list,hedge_iner_posi)[2]
#        sorted_dis_index = [0,2,1]
        print('previous_playlist',previous_playlist)
        print('sorted_dis_index',sorted_dis_index)
        difference_list = list(set(sorted_dis_index[0:2])-set(previous_playlist))
        remove_thread_list = list(set(previous_playlist)-set(sorted_dis_index[0:2]))
        if (not difference_list) == False:
            print('difference/add',difference_list)
            print('remove_thread_list',remove_thread_list)
            for i in range (len(difference_list)):
                add_thread_no = difference_list[i]
                remove_thread_no =remove_thread_list[i]
                print('remove_thread',remove_thread_no)
                threading_list=threading.enumerate()
                for j in range (len(threading_list)):
                    if threading_list[j].name == str(remove_thread_no):
                        threading_list[j].stop()
                        threading_list[j].join(0.1)
                        try:
                            previous_playlist.remove(remove_thread_no)
                        except ValueError:
                            print('previous_playlist',previous_playlist)
                        break
#                        time.sleep(0.2)
                
                sound_thread = Sound(obj_position_list=obj_position_list[add_thread_no],pos_get=pos_get,soundfile=objsound[add_thread_no],HRTF_data=HRTF_data,
                            azimuths=azimuths,padding_len=padding_len,block_size=block_size,
                            buffer_size=buffer_size)
                sound_thread.name = str(add_thread_no)
                sound_thread.start()
                previous_playlist.append(add_thread_no)
        
#        print(threading_list)
        
        time.sleep(1)
        
        
    except KeyboardInterrupt:
        threading_list=threading.enumerate()
        for i in range (len(threading_list)):
            if threading_list[i].name in ['0','1','2', '3', '4', '5', '6', '7']:
                threading_list[i].stop()
                threading_list[i].join(0.1)
        pos_get.stop()
        pos_get.join()
#        socket.close()
        print('done')
        sys.exit()
#sorted_dis_index = np.array([0,2,1])
#previous = [2,1]   
#(not list(set(sorted_dis_index[0:2])-set(previous)))==False
#list(set(previous)-set(sorted_dis_index[0:2]))
#True == False
