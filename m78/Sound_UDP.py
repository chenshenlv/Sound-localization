#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Oct 14 17:26:35 2019

@author: jason
"""

from spatializer import spatialization
from help import help_fun
import time
import math
import numpy as np
import sys
import sounddevice as sd
import soundfile as sf
import queue  # Python 3.x
import threading
from threading import Lock
class Sound(threading.Thread):
    def __init__ (self,obj_position_list=None,pos_get=None,soundfile=None,HRTF_data=None,
                azimuths=None,padding_len=None,block_size=None,
                buffer_size=None):
        self.q=queue.Queue(maxsize=buffer_size)
        self.soundfile=soundfile
        self.block_size=block_size
        self.buffer_size=buffer_size
        self.pos_get = pos_get
        self.padding_len=padding_len
        self.azimuths=azimuths
        self.obj_position_list=obj_position_list
        self.HRTF_data=HRTF_data
        self.terminate = threading.Event()
        self.terminate = False
        self.first_start=True #flag to check if data underflow
        super(Sound, self).__init__()
        
        self.TimeStamp1 = 0
        self.TimeStamp2 = 0
        self.max_len = 6
        self.hp_fun = help_fun()
    def level_sound_buffer(self):
        for i in range(len(self.soundbuffer)):
            self.soundbuffer[i,:]+=self.fraction
        return self.soundbuffer
    def callback(self, outdata, frames, timee, status):

        if self.terminate:
            time.sleep(self.timeout)
            self.q.queue.clear()
            return 0
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
    

    def stop(self):
        self.terminate = True
        
                
    def run(self):
        lock = Lock()
        try:
            self.first_start=True
            
            lock.acquire()
            self.x_out= self.pos_get.pos_proc()   
            lock.release()
                              
            #=========================================================================
            print('x_out',self.x_out)
            hedge_iner_posi=self.hp_fun.y_hedge_axis(self.x_out[0:3],self.x_out[3:6])[1]

            hedge_obj_dis=self.hp_fun.distance(self.obj_position_list,hedge_iner_posi)

            new_y_axis=self.hp_fun.y_hedge_axis(self.x_out[0:3],self.x_out[3:6])[0]
            hedge_obj_vector=np.array(self.obj_position_list)-hedge_iner_posi
            azimuths_body=math.floor(self.hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten()))

    #==============================================================================
            
            with sf.SoundFile(self.soundfile) as f:
                samplerate=f.samplerate 
#                
                for _ in range(self.buffer_size):
                    self.pos=f.tell()
                    f.seek(self.pos)
                    data = f.read(frames=self.block_size, dtype='float32',fill_value=0.0)
            

                    data=data.flatten()                  
                    if len(data)==0:
                        break
                    
                    buffer=spatialization(self.azimuths,self.HRTF_data,data,azimuths_body)*((self.max_len/(hedge_obj_dis+self.max_len))**2)                
                    self.q.put_nowait(buffer)  # Pre-fill queue                
                stream = sd.OutputStream(
                    samplerate=f.samplerate, blocksize=self.block_size-self.padding_len,
                    device=3, channels=2, dtype='float32',
                    callback=self.callback, finished_callback=sd.stop())
                with stream:
                    
                    self.timeout = self.block_size * self.buffer_size / f.samplerate
                    print('timeout',self.timeout)
                    #============================================================
                    while self.terminate:
                        self.q.queue.clear()
                    while not self.terminate :
                        if self.pos<len(f):
                            f.seek(self.pos)
                            for data in f.blocks(blocksize=None, overlap=self.padding_len, frames=-1,
                                dtype='float32',out=data,fill_value=0.0):                                
#                                    print('read position',f.tell())
                                    data=data.flatten()
                                    self.x_out=self.pos_get.pos_proc()
#                                    print ('hedge12_position',x_out)
                                    hedge_iner_posi=self.hp_fun.y_hedge_axis(self.x_out[0:3],self.x_out[3:6])[1]
                                #    hedge_quan=x_out[3:7]
                                    hedge_obj_dis=self.hp_fun.distance(self.obj_position_list,hedge_iner_posi)

                                    new_y_axis=self.hp_fun.y_hedge_axis(self.x_out[0:3],self.x_out[3:6])[0]
                                    hedge_obj_vector=np.array(self.obj_position_list)-hedge_iner_posi
                                    
                                    azimuths_body=self.hp_fun.calculateAzimuths_pos(new_y_axis,hedge_obj_vector.flatten())
#                                    print('azimuths is:',azimuths_body)
                                    buffer=spatialization(self.azimuths,self.HRTF_data,data,azimuths_body)*((self.max_len/(hedge_obj_dis+self.max_len))**2)#spatialize soundfile according to azimuth
#                                    print('alpha:',1/(hedge_obj_dis+1)**2)
                                    self.q.put(buffer, timeout=self.timeout)
                                    self.first_start=False
                                    self.pos=f.tell()
                                    if f.tell()>len(f)-self.block_size:
                                        self.pos=0
#                self.event.wait(2)
        
        except KeyboardInterrupt:
            print('stop')
        except queue.Full:
            print('queue.Full',self.obj_position_list)
