#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  1 20:07:02 2019

@author: jason
"""
from Postion_Process import Postion_Process
from Sound import Sound
from Client import client
from scipy.io import loadmat
from spatializer import spatialization
#from readIMU import hedge_position 
from help import objects, help_fun
from marvelmind import MarvelmindHedge
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
#sd.query_devices(device=17, kind=None)
#sd.default
#class Postion_Process(threading.Thread):
#    def __init__(self, serial=None, x_in=None):
#        self.serial=serial
#        self.TimeStamp1 = 0
#        self.TimeStamp2 = 0
#        self.x_in=x_in
#        self.x_out=np.zeros((1,6),dtype='float32')
#        self.hedge1_pos = np.array([0 , 0 , 0],dtype='float32')
#        self.hedge2_pos = np.array([0 , 0 , 0],dtype='float32')
#        self.terminaterequire = False
#        self.i= 0
#        maxvaluescount = 3
#        self.pos_buffer = collections.deque([[0]*6]*maxvaluescount, maxlen=maxvaluescount)
#        threading.Thread.__init__(self)
#        
#        
#    def pos_proc(self):
##        print('x_out:', self.x_out.flatten())
#        return self.pos_buffer[-1]
#    def stop(self):
#        self.terminaterequire = True         
#    def run(self):
##        for _ in range (2):
#        while (not self.terminaterequire):
#            try:
#                hedge12_position = np.asarray(self.serial.position(),dtype='float32')
##                print ('hedge12_position',serial.position())
#            #                                        if self.serial.position()[0] == 12 and self.TimeStamp1 == hedge12_position[5]:
#            #                                            continue            
#                if hedge12_position[0] == 12 and self.TimeStamp1 < hedge12_position[5]:
#                    self.hedge1_pos = hedge12_position[1:4]
##                    print('hedge1_pos',hedge1_pos)
#                    self.TimeStamp1 = hedge12_position[5]
#                    
#            #                                        if self.serial.position()[0] == 22 and self.TimeStamp1 == hedge12_position[5]:
#            #                                            continue
#                if hedge12_position[0] == 22 and self.TimeStamp2 < hedge12_position[5] :
#                    self.hedge2_pos = hedge12_position[1:4]
#                    self.TimeStamp2 = hedge12_position[5]
#                self.x_in=np.delete(self.x_in,0,0)
##                self.x_in=np.vstack((self.x_in,[self.i+0.2,self.i-0.2,0,self.i-0.11,self.i-0.11,self.i-0.11]))
##                self.x_in=np.vstack((self.x_in,np.hstack((self.hedge1_pos, self.hedge2_pos))))
#                self.x_in=np.vstack((self.x_in,hp_fun.paired_position(self.hedge1_pos, self.hedge2_pos)))
#                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
##                print ('hedge12_position',self.x_out)
#                self.pos_buffer.append(self.x_out)
#                self.i+=0.1
#                time.sleep(0.005)
#            except KeyboardInterrupt:
#                self.stop()
#                sys.exit()
#                break
#                exit
#        
#            except struct.error:
#                self.x_in=np.vstack((self.x_in,[1.1,-2.1,0,0,-3.1,0]))
#            #        self.x_in=np.vstack((self.x_in,np.hstack((self.hedge1_pos, self.hedge2_pos))))
#            #        self.x_in=np.vstack((self.x_in,hp_fun.paired_position(self.hedge1_pos, self.hedge2_pos)))
#                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
#                self.pos_buffer.append(self.x_out)
#      
#        
##            time.sleep(0.001)
#       
#class Sound(threading.Thread):
#    def __init__ (self,obj_position_list=None,serial=None,pos_get=None,soundfile=None,HRTF_data=None,
#                azimuths=None,padding_len=None,block_size=None,
#                buffer_size=None):
#        self.q=queue.Queue(maxsize=buffer_size)
#        self.soundfile=soundfile
#        self.block_size=block_size
#        self.buffer_size=buffer_size
#        self.serial=serial
#        self.pos_get = pos_get
#        self.padding_len=padding_len
#        self.azimuths=azimuths
#        self.obj_position_list=obj_position_list
#        self.HRTF_data=HRTF_data
#        self.terminate = threading.Event()
#        self.terminate = False
#        self.first_start=True #flag to check if data underflow
#        super(Sound, self).__init__()
#        
#        self.TimeStamp1 = 0
#        self.TimeStamp2 = 0
#        self.max_len = 10
#        
#    def level_sound_buffer(self):
#        for i in range(len(self.soundbuffer)):
#            self.soundbuffer[i,:]+=self.fraction
#            
#    #    for i in range(500):
#    #        soundbuffer[-i,:]=soundbuffer[-i,:]*2
#        return self.soundbuffer
#    def callback(self, outdata, frames, timee, status):
#    #    outdata=np.reshape(outdata,(-1,2))
#    #    print('frames',frames)
#    #    print('sizeof outdata',np.shape(outdata))
#        if self.terminate:
#            time.sleep(self.timeout)
#            self.q.queue.clear()
#            return 0
#        if self.first_start:
#            assert frames == self.block_size-self.padding_len
#            if status.output_underflow:
#                print('Output underflow: increase blocksize?', file=sys.stderr)
#    #            raise sd.CallbackStop
#            assert not status
#        try:
#            data=threading.local()
#            data = self.q.get_nowait()
#    #        print('len of data',len(data[:,0]))
#    #        data=data.astype('float64').
#        except queue.Empty:
#            print('Buffer is empty: increase buffersize?', file=sys.stderr)
##            raise sd.CallbackStop
#        if len(data[:,0]) < len(outdata[:,0]):
#            outdata[:len(data[:,0]),0] = data[:,0]
#            outdata[len(data[:,0]):,0] = 0 * (len(outdata[:,0]) - len(data[:,0]))
#            outdata[:len(data[:,0]),1] = data[:,1]
#            outdata[len(data[:,0]):,1] = 0 * (len(outdata[:,0]) - len(data[:,0]))
##            raise sd.CallbackStop
#        else:
#           
#            outdata[:,0] = data[:,0]
#            outdata[:,1] = data[:,1]
#    
#
#    def stop(self):
#        self.terminate = True
#        
#                
#    def run(self):
#        
#        try:
#            self.first_start=True
#            
#            
#            x_out= self.pos_get.pos_proc()                      
#            #=========================================================================
#            
#            hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
##    hedge_quan=x_out[3:7]
#            hedge_obj_dis=hp_fun.distance(self.obj_position_list,hedge_iner_posi)
##            print(hedge_obj_dis)
##            print(self.obj_position_list)
##            hedge_obj_dis_index=hp_fun.distance(self.obj_position_list,hedge_iner_posi)[0]
#        #    rotation_angle_z=hp_fun.quaternionToEulerianAngle(hedge_quan)[2]
#            new_y_axis=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[0]
#            hedge_obj_vector=np.array(self.obj_position_list)-hedge_iner_posi
#            azimuths_body=math.floor(hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten()))
#        #    obj_body=hp_fun.quaternionRotate(hedge_quan,[0,0,-1])
#        #    obj_body=hp_fun.quaternionRotate(hedge_quan,obj_position_list[hedge_obj_dis_index]) #object's position in hedge body system
#        #    azimuths_body=hp_fun.calculateAzimuths(obj_body)
#    #==============================================================================
#            
#            with sf.SoundFile(self.soundfile) as f:
#                samplerate=f.samplerate 
##                
#                for _ in range(self.buffer_size):
#                    self.pos=f.tell()
#                    f.seek(self.pos)
#                    data = f.read(frames=self.block_size, dtype='float32',fill_value=0.0)
#            
#
#                    data=data.flatten()                  
#                    if len(data)==0:
#                        break
#                    
#                    buffer=spatialization(self.azimuths,self.HRTF_data,data,azimuths_body)                 
#                    self.q.put_nowait(buffer)  # Pre-fill queue                
#                stream = sd.OutputStream(
#                    samplerate=f.samplerate, blocksize=self.block_size-self.padding_len,
#                    device=16, channels=2, dtype='float32',
#                    callback=self.callback, finished_callback=sd.stop())
#                with stream:
#                    
#                    self.timeout = self.block_size * self.buffer_size / f.samplerate
#                    print('timeout',self.timeout)
#                    #============================================================
#                    while self.terminate:
#                        self.q.queue.clear()
#                    while not self.terminate :
#                        if self.pos<len(f):
#                            f.seek(self.pos)
#                            for data in f.blocks(blocksize=None, overlap=self.padding_len, frames=-1,
#                                dtype='float32',out=data,fill_value=0.0):                                
##                                    print('read position',f.tell())
#                                    data=data.flatten()
#                                    x_out=self.pos_get.pos_proc()
##                                    print ('hedge12_position',x_out)
#                                    hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
#                                #    hedge_quan=x_out[3:7]
#                                    hedge_obj_dis=hp_fun.distance(self.obj_position_list,hedge_iner_posi)
##                                    print(hedge_obj_dis)
##                                    print(self.obj_position_list)
##                                    hedge_obj_dis_index=hp_fun.distance(self.obj_position_list,hedge_iner_posi)[0]
#                                #    rotation_angle_z=hp_fun.quaternionToEulerianAngle(hedge_quan)[2]
#                                    new_y_axis=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[0]
#                                    hedge_obj_vector=np.array(self.obj_position_list)-hedge_iner_posi
#                                    
#                                    azimuths_body=hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten())
##                                    print ('hedge2_pos',hedge_iner_posi)
##                                    print ('azimuths body',azimuths_body)
##                                    print('y_dot:', new_y_axis)
#                                    buffer=spatialization(azimuths,HRTF_data,data,azimuths_body)/(1-hedge_obj_dis/self.max_len)#spatialize soundfile according to azimuth
#                                    self.q.put(buffer, timeout=self.timeout)
#                                    self.first_start=False
#                                    self.pos=f.tell()
#                                    if f.tell()>len(f)-block_size:
#                                        self.pos=0
##                self.event.wait(2)
#        
#        except KeyboardInterrupt:
#            print('stop')
#        except queue.Full:
#            print('queue.Full',self.obj_position_list)
            

    
    
hp_fun=help_fun()
obj1 = objects(0, 0 ,0, './SoundTrack/SoftBall_Charles_DeTore--4db.wav')
obj2 = objects(4.420, -1.59, 0, './SoundTrack/RiverStreamAdjusted--4db.wav')
obj3 = objects(-1.504, -5.054, 0, './SoundTrack/mono2002-11-22t04--4db.wav')
#obj1 = objects(-2.0, 0.5 ,0, './SoundTrack/mono2002-11-22t04.wav')
#obj2 = objects(3.0, -0.5, 0, './SoundTrack/RiverStreamAdjusted.wav')
#obj3 = objects(-0.5, -5, 0, './SoundTrack/Mono2010-03-17tr01.wav')
obj_position_list=[obj1.position, obj2.position,  obj3.position]
objsound=[obj1.soundfile, obj2.soundfile, obj3.soundfile]
HRTF_data = loadmat('./HRTF/147_Propcessed_HRTF.mat')
azimuths =HRTF_data['azimuths'].flatten()
azimuths = azimuths.tolist()
previous_playlist = []
#azimuths = [-80, -65, -55, -45, -40, -35, -30, -25, -20,
#                -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 55, 65, 80]
padding_len = len(np.squeeze(HRTF_data['hrir_l'][0, :]))-1 # 200*1
# 50 elevation degrees
#elevations = [-45+5.625*e for e in range(50)]
#event = threading.Event() 
#ip_add='192.168.1.15'
#port = 8080
#socket = client()
#socket.connect(ip_add,port)
time.sleep(2)
block_size = 10000 #frames size Sampling frequency in Hertz (= frames per second).
buffer_size = 8 #number of blocks used for buffering
    
serial = MarvelmindHedge(adr=None, tty="/dev/ttyACM0", baud=115200, debug=False) # create MarvelmindHedge thread
serial.start() # start thread
time.sleep(1)
TimeStamp1 = 0  #initiate position thread and buffer
TimeStamp2 = 0
hedge1_pos = np.array([0 , 0 , 0],dtype='float32')
hedge2_pos = np.array([0 , 0 , 0],dtype='float32')
x_in=np.zeros((3,6),dtype='float32')
for i in range (len(x_in)):
    for _ in range (2):
        hedge12_position = np.asarray(serial.position(),dtype='float32')
#                    if self.serial.position()[0] == 12 and self.TimeStamp1 == hedge12_position[5]:
#                        continue            
        if hedge12_position[0] == 12 and TimeStamp1 < hedge12_position[5]:
            hedge1_pos =hedge12_position[1:4]
            TimeStamp1 = hedge12_position[5]
            
#                    if self.serial.position()[0] == 22 and self.TimeStamp1 == hedge12_position[5]:
#                        continue
        if hedge12_position[0] == 22 and TimeStamp2 < hedge12_position[5] :
            hedge2_pos = hedge12_position[1:4]
            TimeStamp2 = hedge12_position[5]
            
            
    x_in[i,:]=hp_fun.paired_position(hedge1_pos, hedge2_pos) 
#    time.sleep(0.11)               
pos_get = Postion_Process(serial=serial, x_in=x_in)
pos_get.start()
time.sleep(1)
x_out=pos_get.pos_proc()
hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
sorted_dis_index = hp_fun.distance(obj_position_list,hedge_iner_posi)[2]
for i in range(2):
    sound_thread = Sound(obj_position_list=obj_position_list[sorted_dis_index[i]],serial=serial,pos_get=pos_get,soundfile=objsound[sorted_dis_index[i]],HRTF_data=HRTF_data,
                                azimuths=azimuths,padding_len=padding_len,block_size=block_size,
                                buffer_size=buffer_size)
    sound_thread.name = str(i)
    sound_thread.start()
    previous_playlist.append(i)
time.sleep(2)    
   
while True:
    try:

        x_out=pos_get.pos_proc()
        hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
#        socket.send(str(hedge_iner_posi).encode('utf-8'))
        print('hedge_iner_posi',hedge_iner_posi)
#        hedge_quan=x_out[3:7]
#        hedge_quan=np.array([0.0122,0.089,0.202,0.0455])
        thread_switcher_array=hp_fun.distance(obj_position_list,hedge_iner_posi)[0]
#        print(thread_switcher_array)
        sorted_dis = hp_fun.distance(obj_position_list,hedge_iner_posi)[1]
        sorted_dis_index = hp_fun.distance(obj_position_list,hedge_iner_posi)[2]
#        sorted_dis_index = [0,2,1]
#        print('sorted_dis_index',sorted_dis_index)
        if len(list(set(sorted_dis_index[0:2])-set(previous_playlist)))==1:
            add_thread_no = list(set(sorted_dis_index[0:2])-set(previous_playlist))[0]
            remove_thread_no =list(set(previous_playlist)-set(sorted_dis_index[0:2]))[0]
            threading_list=threading.enumerate()
            for i in range (len(threading_list)):
                if threading_list[i].name == str(remove_thread_no):
                    threading_list[i].stop()
                    threading_list[i].join(0.1)
                    time.sleep(2)
                    previous_playlist.remove(remove_thread_no)
                    sound_thread = Sound(obj_position_list=obj_position_list[add_thread_no],serial=serial,pos_get=pos_get,soundfile=objsound[add_thread_no],HRTF_data=HRTF_data,
                                azimuths=azimuths,padding_len=padding_len,block_size=block_size,
                                buffer_size=buffer_size)
                    sound_thread.name = str(add_thread_no)
                    sound_thread.start()
                    previous_playlist.append(add_thread_no)
#        print('previous_playlist',previous_playlist)
#        print(threading_list)
        
        time.sleep(0.3)
        
        
    except KeyboardInterrupt:
        threading_list=threading.enumerate()
        for i in range (len(threading_list)):
            if threading_list[i].name in ['0','1','2']:
                threading_list[i].stop()
                threading_list[i].join(0.1)
        pos_get.stop()
        pos_get.join()
        serial.stop()
        socket.close()
        print('done')
        sys.exit()


