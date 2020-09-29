#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun  5 16:23:04 2019

@author: jason
"""
#import scipy
from scipy.io import loadmat
from spatializer import spatialization
#from readIMU import hedge_position 
from help import objects, help_fun
from marvelmind import MarvelmindHedge
import time
import math
import numpy as np
import multiprocessing as mp
#import random
import sys
#import matplotlib.pyplot as plt
#import time as time
import sounddevice as sd
import soundfile as sf
import queue  # Python 3.x
import threading


hp_fun=help_fun()

obj1 = objects(0, 0, 0, './SoundTrack/Mono2010-03-17tr01.wav')
obj_position_list=[obj1.position]

HRTF_data = loadmat('./HRTF/58_Propcessed_HRTF.mat')
azimuths =HRTF_data['azimuths'].flatten()
azimuths = azimuths.tolist()
#azimuths = [-180, -150, -120, -90, -60, -30, 0, 30, 60, 90, 120, 150]
#azimuths = [-80, -65, -55, -45, -40, -35, -30, -25, -20,
#                -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 55, 65, 80]
boundary_len = len(np.squeeze(HRTF_data['hrir_l'][0, :]))-1 # 200*1

#obj1 = objects(2,0,0,'i_ran_so_far_away-flock_of_seagulls.wav')
#
#obj_position_list=[obj1.position]
#
#azimuths = [-80, -65, -55, -45, -40, -35, -30, -25, -20,
#                -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 55, 65, 80]


# 50 elevation degrees
#elevations = [-45+5.625*e for e in range(50)]
#HRTF_data = loadmat('./CIPIC_58_HRTF.mat')
filename = obj1.soundfile   

block_size = 5000 #frames size Sampling frequency in Hertz (= frames per second).

#block_size = 7168 #frames size Sampling frequency in Hertz (= frames per second).
buffer_size = 3 #number of blocks used for buffering    
q = queue.Queue(maxsize=buffer_size)
event = threading.Event()


def level_sound_buffer(soundbuffer,fraction):
    for i in range(len(fraction)):
        soundbuffer[i,:]-=fraction[i,:]
        
#    for i in range(500):
#        soundbuffer[i,:]=soundbuffer[i,:]*np.exp(i-500)
#        soundbuffer[-i,:]=soundbuffer[-i,:]*np.exp(-i)
    return soundbuffer
def callback(outdata, frames, time, status):
#    outdata=np.reshape(outdata,(-1,2))
#    print('frames',frames)
#    print('sizeof outdata',np.shape(outdata))
    assert frames == block_size-boundary_len
    if status.output_underflow:
        print('Output underflow: increase blocksize?', file=sys.stderr)
        raise sd.CallbackAbort
    assert not status
    try:
        data = q.get_nowait()
#        print('len of data',len(data[:,0]))
#        data=data.astype('float64').
    except queue.Empty:
        print('Buffer is empty: increase buffersize?', file=sys.stderr)
        raise sd.CallbackAbort
    if len(data[:,0]) < len(outdata[:,0]):
        outdata[:len(data[:,0]),0] = data[:,0]
        outdata[len(data[:,0]):,0] = 0 * (len(outdata[:,0]) - len(data[:,0]))
        outdata[:len(data[:,0]),1] = data[:,1]
        outdata[len(data[:,0]):,1] = 0 * (len(outdata[:,0]) - len(data[:,0]))
        raise sd.CallbackStop
    else:
       
        outdata[:,0] = data[:,0]
        outdata[:,1] = data[:,1]
        
       
try:
    serial = MarvelmindHedge(adr=None, tty="/dev/ttyACM1", baud=115200, debug=False) # create MarvelmindHedge thread
    serial.start() # start thread0
    
#    serial_2 = MarvelmindHedge(adr=22, tty="/dev/ttyACM0", baud=115200, debug=False) # create MarvelmindHedge thread
#    serial_2.start() # start thread0

    x_in=np.zeros((20,7),dtype='float32')
    x_out=np.zeros((1,7),dtype='float32')
    buffer_plot=np.zeros((1,2),dtype='float32')
    for i in range (len(x_in)):
        x_in[i,:]=serial.valuesImuData[-1][0:7] 
        time.sleep(0.001)               
    x_out= np.sum(x_in,axis=0)/np.shape(x_in)[0]
    
    hedge_iner_posi=x_out[0:3]
    hedge_quan=x_out[3:7]
    hedge_obj_dis=hp_fun.distance(obj_position_list,hedge_iner_posi)[1]
    hedge_obj_dis_index=hp_fun.distance(obj_position_list,hedge_iner_posi)[0]
    rotation_angle_z=hp_fun.quaternionToEulerianAngle(hedge_quan)[2]
    new_y_axis=hp_fun.y_hedge_axis_Angle_z(rotation_angle_z)
    hedge_obj_vector=hedge_iner_posi-np.array(obj_position_list)
    
    azimuths_body=math.floor(hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten()))
#    obj_body=hp_fun.quaternionRotate(hedge_quan,[0,0,-1])
#    obj_body=hp_fun.quaternionRotate(hedge_quan,obj_position_list[hedge_obj_dis_index]) #object's position in hedge body system
#    azimuths_body=hp_fun.calculateAzimuths(obj_body)

    with sf.SoundFile(filename) as f:
#        sd.query_devices(device=18, kind=None)
#        sd.default
#        routine=0
        
        data = f.read(frames=block_size, dtype='float32',fill_value=0.0)
#        print('read position',f.tell())
        data=data.flatten()

        samplerate=f.samplerate 
        buffer_plot=np.zeros((1,2),dtype='float32')
#        for i in range(buffer_size):
#
#        x_in=np.zeros((30,7),dtype='float32')
#        x_out=np.zeros((1,7),dtype='float32')
#        for i in range (len(x_in)):
#            time.sleep(0.001)
#            x_in[i,:]=serial.valuesImuData[-1][0:7]                
#        x_out= np.sum(x_in,axis=0)/30
#        
#        hedge_iner_posi=x_out[0:3]
#        hedge_quan=x_out[3:7]
#        hedge_obj_dis=hp_fun.distance(obj_position_list,hedge_iner_posi)[1]
#        hedge_obj_dis_index=hp_fun.distance(obj_position_list,hedge_iner_posi)[0]
#        obj_body=hp_fun.quaternionRotate(hedge_quan,obj_position_list[hedge_obj_dis_index]) #object's position in hedge body system
#        azimuths_body=hp_fun.calculateAzimuths(obj_body) 
#        time.sleep(0.01)
        for _ in range(buffer_size):

            
            if len(data)==0:
                break
            
            buffer=spatialization(azimuths,HRTF_data,data,azimuths_body)
           
            q.put_nowait(buffer)  # Pre-fill queue
        boundary_point=buffer[-51:-1,:] #to level the boundary between buffers
        stream = sd.OutputStream(
            samplerate=f.samplerate, blocksize=block_size-boundary_len,clip_off=False,dither_off=True,
            device=16, channels=2, dtype='float32',
            callback=callback, finished_callback=event.set)
        with stream:
            timeout = block_size * buffer_size / f.samplerate
            while len(data)>0:
                sudo_angle=-90
                for data in f.blocks(blocksize=None, overlap=boundary_len, frames=-1,
                    dtype='float32',out=data,fill_value=0.0):
                        print('read position',f.tell())
                        data=data.flatten()
#                            data = f.read(frames=block_size, dtype='float32',fill_value=0.0)
                        x_in=np.delete(x_in,0,0)
                        x_in=np.vstack((x_in,serial.valuesImuData[-1][0:7]))
                        time.sleep(0.001)
                        x_out=np.sum(x_in,axis=0)/np.shape(x_in)[0]
                        hedge_iner_posi=x_out[0:3]
                        hedge_quan=x_out[3:7]
                        hedge_obj_dis=hp_fun.distance(obj_position_list,hedge_iner_posi)[1]
                        hedge_obj_dis_index=hp_fun.distance(obj_position_list,hedge_iner_posi)[0]
                        rotation_angle_z=hp_fun.quaternionToEulerianAngle(hedge_quan)[2]
                        new_y_axis=hp_fun.y_hedge_axis_Angle_z(rotation_angle_z)
                        hedge_obj_vector=hedge_iner_posi-np.asarray(obj_position_list)
#                        new_y_axis=hp_fun.y_hedge_axis(sudo_angle) #sudo test code
#                        hedge_obj_vector=np.array([1,0,0])-np.asarray(obj_position_list) #sudo test code 
                        azimuths_body=hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten())
#                        obj_body=hp_fun.quaternionRotate(hedge_quan,[obj_position_list[0][0],obj_position_list[0][1],1])
#                        obj_body=hp_fun.quaternionRotate(hedge_quan,obj_position_list[hedge_obj_dis_index]) #object's position in hedge body system
#                        azimuths_body=hp_fun.calculateAzimuths([obj_position_list[0][0]-hedge_iner_posi[0],obj_position_list[0][1]-hedge_iner_posi[1],obj_position_list[0][2]-hedge_iner_posi[2]]) 
                        X,Y,Z=hp_fun.quaternionToEulerianAngle(hedge_quan)
                        print ('distance between hedge-obj',hedge_obj_dis)
                        print ('azimuths body',azimuths_body)
                        print('Euler angle:', X,Y,Z)
                        sudo_angle+=2
                        if sudo_angle==270:
                            sudo_angle=-90
                        buffer=spatialization(azimuths,HRTF_data,data,azimuths_body)#/np.max((math.sqrt(hedge_obj_dis),1))#spatialize soundfile according to azimuth
                        fraction=boundary_point-buffer[0:50,:]
        #                buffer=level_sound_buffer(buffer,fraction)
        #                time.sleep(0.1)
                        boundary_point=buffer[-51:-1,:] #to level the boundary between buffers
                        
                        buffer_plot=np.vstack((buffer_plot,buffer))
                        q.put(buffer, timeout=timeout)
        event.wait(2)  # Wait until playback is finished
except KeyboardInterrupt:
    serial.stop()
    sys.exit()

#                data = f.read(frames=block_size, dtype='float32',fill_value=0.0)
#                x_in=np.delete(x_in,0,0)
#                x_in=np.vstack((x_in,serial.valuesImuData[-1][0:7]))
#                x_out=np.sum(x_in,axis=0)/30
#                hedge_iner_posi=x_out[0:3]
#                hedge_quan=x_out[3:7]
#                hedge_obj_dis=hp_fun.distance(obj_position_list,hedge_iner_posi)[1]
#                hedge_obj_dis_index=hp_fun.distance(obj_position_list,hedge_iner_posi)[0]
#                obj_body=hp_fun.quaternionRotate(hedge_quan,obj_position_list[hedge_obj_dis_index]) #object's position in hedge body system
#                azimuths_body=hp_fun.calculateAzimuths(obj_body) 
#                print ('azimuths body',azimuths_body)
#                buffer=spatialization(azimuths,HRTF_data,data,azimuths_body)
#                q.put(buffer, timeout=timeout)
#            event.wait(2)  # Wait until playback is finished

#except queue.Full:
#    print('timeout')
#    exit


#import matplotlib.pyplot as plt
#plt.plot(buffer_plot[4000:4200,0])
#X,Y,Z=hp_fun.quaternionToEulerianAngle([0.707106,0,0,-0.707106])
#hp_fun.quaternionRotate([0.707106,0,0,0.707106], [2,2,0])
#new_y_axis=hp_fun.y_hedge_axis(Z)
#hedge_obj_vector=np.array([-1,-1,0])
#azimuths_body=hp_fun.calculateAzimuths(new_y_axis,hedge_obj_vector)