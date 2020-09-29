#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Jul  1 20:07:02 2019

@author: jason
"""

#from __future__ import print_function
#import sounddevice as sd
#from numpy import pi, sin, arange
#import threading
#import time
#
#def streamer(stream, data):
#    stream.start()
#    stream.write(data)
#    stream.close()
#
#for i in range(1,7):
#    x = 0.1*sin(2*pi*i*440*arange(96e3)/48e3, dtype='float32')
#    stream = sd.OutputStream(device='default', samplerate=48000, channels=1, dtype='float32')
#    thread = threading.Thread(target=streamer, args=(stream, x))
#    thread.start()
#    print('Sin %dHz' % (i*440))
#    time.sleep(0.5)
#thread.join()



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
class Postion_Process(threading.Thread):
    def __init__(self, serial=None, x_in=None):
        self.serial=serial
        self.TimeStamp1 = 0
        self.TimeStamp2 = 0
        self.x_in=x_in
        self.x_out=np.zeros((1,6),dtype='float32')
        self.hedge1_pos = np.array([0 , 0 , 0],dtype='float32')
        self.hedge2_pos = np.array([0 , 0 , 0],dtype='float32')
        self.terminaterequire = False
        self.i= -0.1
        maxvaluescount = 3
        self.pos_buffer = collections.deque([[0]*6]*maxvaluescount, maxlen=maxvaluescount)
        threading.Thread.__init__(self)
        
        
    def pos_proc(self):
#        print('x_out:', self.x_out.flatten())
        return self.pos_buffer[-1]
    def stop(self):
        self.terminaterequire = True         
    def run(self):
#        for _ in range (2):
        while (not self.terminaterequire):
            try:
                hedge12_position = np.asarray(self.serial.position(),dtype='float32')
#                print ('hedge12_position',hedge12_position)
            #                                        if self.serial.position()[0] == 12 and self.TimeStamp1 == hedge12_position[5]:
            #                                            continue            
                if hedge12_position[0] == 12 and self.TimeStamp1 < hedge12_position[5]:
                    self.hedge1_pos = hedge12_position[1:4]
            #            print('hedge1_pos',self.hedge1_pos)
                    self.TimeStamp1 = hedge12_position[5]
                    
            #                                        if self.serial.position()[0] == 22 and self.TimeStamp1 == hedge12_position[5]:
            #                                            continue
                if hedge12_position[0] == 22 and self.TimeStamp2 < hedge12_position[5] :
                    self.hedge2_pos = hedge12_position[1:4]
                    self.TimeStamp2 = hedge12_position[5]
                self.x_in=np.delete(self.x_in,0,0)
                self.x_in=np.vstack((self.x_in,[0,0,0,0,0,0]))
#                self.x_in=np.vstack((self.x_in,np.hstack((self.hedge1_pos, self.hedge2_pos))))
#                self.x_in=np.vstack((self.x_in,hp_fun.paired_position(self.hedge1_pos, self.hedge2_pos)))
                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
                self.pos_buffer.append(self.x_out)
                self.i+=0.001
                
            except KeyboardInterrupt:
                self.stop()
                sys.exit()
                break
                exit
        
            except struct.error:
                self.x_in=np.vstack((self.x_in,[1.1,-2.1,0,0,-3.1,0]))
            #        self.x_in=np.vstack((self.x_in,np.hstack((self.hedge1_pos, self.hedge2_pos))))
            #        self.x_in=np.vstack((self.x_in,hp_fun.paired_position(self.hedge1_pos, self.hedge2_pos)))
                self.x_out=np.sum(self.x_in,axis=0)/np.shape(self.x_in)[0]
                self.pos_buffer.append(self.x_out)
      
        
#            time.sleep(0.001)
       
class Sound(threading.Thread):
    def __init__ (self,stop_event=None,obj_position_list=None,serial=None,pos_get=None,que=None,soundfile=None,HRTF_data=None,
                azimuths=None,padding_len=None,block_size=None,
                buffer_size=None):
        self.q=que
        self.soundfile=soundfile
        self.block_size=block_size
        self.buffer_size=buffer_size
        self.serial=serial
        self.pos_get = pos_get
        self.padding_len=padding_len
        self.azimuths=azimuths
        self.obj_position_list=obj_position_list
        self.HRTF_data=HRTF_data
        self.stop_event=stop_event
        self.first_start=True #flag to check if data underflow
#        threading.Thread.__init__(self)
        super(Sound, self).__init__()
        self._stop_event = threading.Event()
        self.event = threading.Event()
        self.TimeStamp1 = 0
        self.TimeStamp2 = 0
        self.terminate = False
    def level_sound_buffer(self):
        for i in range(len(self.soundbuffer)):
            self.soundbuffer[i,:]+=self.fraction
            
    #    for i in range(500):
    #        soundbuffer[-i,:]=soundbuffer[-i,:]*2
#        return self.soundbuffer
    def stop(self):
        self.terminate = True
        self._stop_event.clear()
        
    def callback(self, outdata, frames, time, status):
    #    outdata=np.reshape(outdata,(-1,2))
    #    print('frames',frames)
    #    print('sizeof outdata',np.shape(outdata))
        if self.terminate:
            return True
#        self.stop_event.wait()
        
        if self.first_start:
            assert frames == self.block_size-self.padding_len
            if status.output_underflow:
                print('Output underflow: increase blocksize?', file=sys.stderr)
    #            raise sd.CallbackStop
            assert not status
        try:
            data=threading.local()
            data = self.q.get_nowait()
    #        print('len of data',len(data[:,0]))
    #        data=data.astype('float64').
        except queue.Empty:
            print('Buffer is empty: increase buffersize?', file=sys.stderr)
#            raise sd.CallbackStop
        if len(data[:,0]) < len(outdata[:,0]):
            outdata[:len(data[:,0]),0] = data[:,0]
            outdata[len(data[:,0]):,0] = 0 * (len(outdata[:,0]) - len(data[:,0]))
            outdata[:len(data[:,0]),1] = data[:,1]
            outdata[len(data[:,0]):,1] = 0 * (len(outdata[:,0]) - len(data[:,0]))
#            raise sd.CallbackStop
        else:
           
            outdata[:,0] = data[:,0]
            outdata[:,1] = data[:,1]
    

            
    def run(self):
        
        try:
            self.first_start=True
            
            
            x_out= self.pos_get.pos_proc()                      
            #=========================================================================
            
            hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
#    hedge_quan=x_out[3:7]
            hedge_obj_dis=hp_fun.distance(self.obj_position_list,hedge_iner_posi)
#            print(hedge_obj_dis)
#            print(self.obj_position_list)
#            hedge_obj_dis_index=hp_fun.distance(self.obj_position_list,hedge_iner_posi)[0]
        #    rotation_angle_z=hp_fun.quaternionToEulerianAngle(hedge_quan)[2]
            new_y_axis=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[0]
            hedge_obj_vector=np.array(self.obj_position_list)-hedge_iner_posi
            azimuths_body=math.floor(hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten()))
        #    obj_body=hp_fun.quaternionRotate(hedge_quan,[0,0,-1])
        #    obj_body=hp_fun.quaternionRotate(hedge_quan,obj_position_list[hedge_obj_dis_index]) #object's position in hedge body system
        #    azimuths_body=hp_fun.calculateAzimuths(obj_body)
    #==============================================================================
            
            with sf.SoundFile(self.soundfile) as f:
                samplerate=f.samplerate 
#                
                for _ in range(self.buffer_size):
                    pos=f.tell()
                    f.seek(pos)
                    data = f.read(frames=self.block_size, dtype='float32',fill_value=0.0)
            

                    data=data.flatten()                  
                    if len(data)==0:
                        break
                    
                    buffer=spatialization(self.azimuths,self.HRTF_data,data,azimuths_body)                 
                    self.q.put_nowait(buffer)  # Pre-fill queue   
                if self.terminate:
                    return True
                stream = sd.OutputStream(
                    samplerate=f.samplerate, blocksize=self.block_size-self.padding_len,
                    device=1, channels=2, dtype='float32',
                    callback=self.callback, finished_callback=self.event.set())
                with stream:
                    
                    timeout = self.block_size * self.buffer_size / f.samplerate
                    print('timeout',timeout)
                    if self.terminate:
                        return True
                    #============================================================
                    while True :
                        if self.terminate:
                            break
                        if pos<len(f):
                            f.seek(pos)
                            for data in f.blocks(blocksize=None, overlap=self.padding_len, frames=-1,
                                dtype='float32',out=data,fill_value=0.0):                                
                                    print('read position',f.tell())
                                    data=data.flatten()
#                                    x_out=self.pos_get.pos_proc()
                                    x_out=np.array([0,0,0,0,0,0])
                                    hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
                                #    hedge_quan=x_out[3:7]
                                    hedge_obj_dis=hp_fun.distance(self.obj_position_list,hedge_iner_posi)
#                                    print(hedge_obj_dis)
#                                    print(self.obj_position_list)
#                                    hedge_obj_dis_index=hp_fun.distance(self.obj_position_list,hedge_iner_posi)[0]
                                #    rotation_angle_z=hp_fun.quaternionToEulerianAngle(hedge_quan)[2]
                                    new_y_axis=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[0]
                                    hedge_obj_vector=np.array(self.obj_position_list)-hedge_iner_posi
                                    
                                    azimuths_body=hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten())
                                    print ('hedge2_pos',hedge_iner_posi)
                                    print ('azimuths body',azimuths_body)
                                    print('y_dot:', new_y_axis)
                                    buffer=spatialization(azimuths,HRTF_data,data,azimuths_body)#/np.max((math.sqrt(hedge_obj_dis),1))#spatialize soundfile according to azimuth
                                    self.q.put(buffer, timeout=timeout)
                                    self.first_start=False
                                    pos=f.tell()
                                    if f.tell()>len(f)-block_size:
                                        pos=0
                
                self.event.wait(2)
        
        except KeyboardInterrupt:
            self.serial.stop()
            self.stop_event.clear()
            self.stop()
            stream.close()
            f.close()
            
            
            
        except queue.Full:
            print('timeout')
            exit

    
hp_fun=help_fun()
obj1 = objects(5.828, 9 ,0, './SoundTrack/mono2002-11-22t04.wav')
obj2 = objects(0.5, 0.2, 0, './SoundTrack/RiverStreamAdjusted.wav')
obj3 = objects(120, -5.37, 0, './SoundTrack/Mono2010-03-17tr01.wav')
obj_position_list=[obj1.position, obj2.position,  obj3.position]
objsound=[obj1.soundfile, obj2.soundfile, obj3.soundfile]
HRTF_data = loadmat('./HRTF/147_Propcessed_HRTF.mat')
azimuths =HRTF_data['azimuths'].flatten()
azimuths = azimuths.tolist()
#azimuths = [-80, -65, -55, -45, -40, -35, -30, -25, -20,
#                -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 55, 65, 80]
padding_len = len(np.squeeze(HRTF_data['hrir_l'][0, :]))-1 # 200*1
# 50 elevation degrees
#elevations = [-45+5.625*e for e in range(50)]
#event = threading.Event() 
   
block_size = 10000 #frames size Sampling frequency in Hertz (= frames per second).
buffer_size = 10 #number of blocks used for buffering
buffer_que=[]
stop_event=[]
sound_thread=[]
for i in range(2):
    buffer_que.append(queue.Queue(maxsize=buffer_size))
    stop_event.append(threading.Event())
    stop_event[i].clear()
serial = MarvelmindHedge(adr=None, tty="/dev/ttyACM0", baud=115200, debug=False) # create MarvelmindHedge thread
serial.start() # start thread
time.sleep(1)
TimeStamp1 = 0  #initiate position thread and buffer
TimeStamp2 = 0
hedge1_pos = np.array([0 , 0 , 0],dtype='float32')
hedge2_pos = np.array([0 , 0 , 0],dtype='float32')
x_in=np.zeros((2,6),dtype='float32')
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
   
for i in range(1):
    thread = Sound(stop_event=stop_event[i],obj_position_list=obj_position_list[i],serial=serial,pos_get=pos_get,que=buffer_que[i],soundfile=objsound[i],HRTF_data=HRTF_data,
                azimuths=azimuths,padding_len=padding_len,block_size=block_size,
                buffer_size=buffer_size)
#    thread.daemon = True
    thread.start()

    sound_thread.append(thread)
    stop_event[0].set()
    time.sleep(0.1)      
while True:
    try:
        
        x_out=pos_get.pos_proc()
        hedge_iner_posi=hp_fun.y_hedge_axis(x_out[0:3],x_out[3:6])[1]
#        hedge_quan=x_out[3:7]
#        hedge_quan=np.array([0.0122,0.089,0.202,0.0455])
        thread_switcher_array=hp_fun.distance(obj_position_list,hedge_iner_posi)[0]
        print(thread_switcher_array)
        sorted_dis = hp_fun.distance(obj_position_list,hedge_iner_posi)[1]
        sorted_dis_index = hp_fun.distance(obj_position_list,hedge_iner_posi)[2]
        
#        stop_event[1].set()
#        for i in range(3):
#            if thread_switcher_array[i]==0:
#                stop_event[i].clear()
#            elif thread_switcher_array[i]==1:
#                stop_event[i].set()
        
        
        
    except KeyboardInterrupt:
        pos_get.stop()
        pos_get.join()
        serial.stop()
        serial.join()
        for i in range(1):
            sound_thread[i].stop_event.set()
            sound_thread[i].stop()
            sound_thread[i].join()
        for i in range (1):
            if sound_thread[i].is_alive():
                print ('thread %s is alive',i)
                time.sleep(0.5)
        print('done')
        sys.exit()



         
