#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 21:52:40 2019

@author: jason
"""

from scipy.io import loadmat,savemat
from operator import itemgetter
import numpy as np
#
#HRTF_data = loadmat('./KEMAR_MED2_2017_DEC-22.mat')
#azimuths=np.reshape(HRTF_data['azimuth_array'],(12))
#elevation=np.reshape(HRTF_data['elevation_array'],(6))
#HRIR_data=HRTF_data['rawData'][:,:,24:24+12]
#HRIR_data=np.reshape(HRIR_data,(2,12,1024))
#hrir_l=HRIR_data[0,:,:]
#hrir_r=HRIR_data[1,:,:]
#ITD=np.reshape(HRTF_data['tdData'][0,24:24+12],(12))
#for i in range(len(ITD)): 
#    if ITD[i]>0:
#        hrir_r[i,0:ITD[i]]=0
#    elif ITD[i]<0:
#        hrir_l[i,0:-ITD[i]]=0
#savemat('./Propcessed_HRTF.mat', {'ITD':ITD,'hrir_l':hrir_l,'hrir_r':hrir_r,'azimuths':azimuths,
#                                'elevation':elevation},oned_as='column')
#test=loadmat('Propcessed_HRTF.mat')
#delay=test['ITD']

#=========================================================#
#=========================================================#
#=========================================================#
#=========================================================#
#
#HRTF_data = loadmat('./large_pinna_final.mat')
#azimuths=np.zeros((72))
#for i in range(72):
#    azimuths[i]=-180+i*5
##azimuths=np.reshape(HRTF_data['azimuth_array'],(12))
##elevation=np.reshape(HRTF_data['elevation_array'],(6))
##HRIR_data=HRTF_data['rawData'][:,:,36:36+12]
##HRIR_data=np.reshape(HRIR_data,(2,12,1024))
#hrir_l=np.reshape(HRTF_data['left'],(72,200))
#hrir_r=np.reshape(HRTF_data['right'],(72,200))
#temp=np.zeros((72,200))
#temp[0:36,:] = hrir_l[36:72,:]
#temp[36:72,:] = hrir_l[0:36,:]
#hrir_l=temp
#temp=np.zeros((72,200))
#temp[0:36,:] = hrir_r[36:72,:]
#temp[36:72,:] = hrir_r[0:36,:]
#hrir_r = temp
#
#savemat('special_Propcessed_HRTF.mat', {'hrir_l':hrir_l,'hrir_r':hrir_r,'azimuths':azimuths,
#                                },oned_as='column')
#test=loadmat('special_Propcessed_HRTF.mat')
#=========================================================#
#=========================================================#
#=========================================================#
#=========================================================#

num=0
for i in range (166):
    try:
        
        HRTF_data = loadmat('./RawHRTF/CIPIC_'+str(i)+'_HRTF.mat')
        azimuths = [-80, -65, -55, -45, -40, -35, -30, -25, -20,
                        -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 55, 65, 80]
        
        hrir_l_front = np.squeeze(HRTF_data['hrir_l'][0:25, 8, :])  # 200*1
        hrir_l_rear = np.squeeze(HRTF_data['hrir_l'][0:25, 40, :])  # 200*1
        hrir_r_front = np.squeeze(HRTF_data['hrir_r'][0:25, 8, :])  # 200*1
        hrir_r_rear = np.squeeze(HRTF_data['hrir_r'][0:25, 40, :])  # 200*1
        itd_front = HRTF_data['ITD'][0:25, 8]
        itd_rear = HRTF_data['ITD'][0:25, 40]
        temp=np.zeros((50,200))
        temp[24:37,:] = hrir_l_front[12:25,:]
        temp[37:50,:] = hrir_l_rear[-1:-14:-1,:]
        temp[0:12,:] = hrir_l_rear[-14:-26:-1,:]
        temp[12:24,:] = hrir_l_front[0:12:,:]
        
        #temp[24:37,:] = hrir_l_front[12:25,:]
        #temp[37:50,:] = hrir_l_rear[0:13,:]
        #temp[0:12,:] = hrir_l_rear[13:25,:]
        #temp[12:24,:] = hrir_l_front[0:12,:]
        hrir_l=temp
        
        temp=np.zeros((50,200))
        temp[24:37,:] = hrir_r_front[12:25,:]
        temp[37:50,:] = hrir_r_rear[-1:-14:-1,:]
        temp[0:12,:] = hrir_r_rear[-14:-26:-1,:]
        temp[12:24,:] = hrir_r_front[0:12:,:]
        hrir_r = temp
        
        #temp[24:37,:] = hrir_r_front[12:25,:]
        #temp[37:50,:] = hrir_r_rear[0:13,:]
        #temp[0:12,:] = hrir_r_rear[13:25,:]
        #temp[12:24,:] = hrir_r_front[0:12,:]
        
        itd = np.zeros((50))
        itd[24:37] = itd_front[12:25]
        itd[37:50] = itd_rear[-1:-14:-1]
        itd[0:12] = itd_rear[-14:-26:-1]
        itd[12:24] = itd_front[0:12:]
        
        azimuths = [-175, -170, -165, -160, -155, -150, -145, -140, -135, -125, -115, -100, 
                    -80, -65, -55, -45, -40, -35, -30, -25, -20, -15, -10, -5,
                    0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 55, 65, 80,
                    100, 115, 125, 135, 140, 145, 150, 155, 160, 165, 170, 175, 180]
        savemat('./'+str(i)+'_Propcessed_HRTF.mat', {'hrir_l':hrir_l,'hrir_r':hrir_r,'azimuths':azimuths,
                                        'itd':itd},oned_as='column')
        num+=1
    except FileNotFoundError:
        continue
        
energy = np.zeros((num,2))
delay = np.zeros((num,2))
num=0
for i in range (166):
    try:
   
        data = loadmat('./'+str(i)+'_Propcessed_HRTF.mat')
        hrir = data['hrir_l']
        hrir_abs = np.absolute(hrir)
        hrir_sum = np.sum(hrir_abs)
        hrir = data['hrir_r']
        hrir_abs = np.absolute(hrir)
        hrir_sum+= np.sum(hrir_abs)
        energy[num,0] = i
        energy[num,1] = hrir_sum
        
        itd = data['itd']
        itd_abs = np.absolute(itd)
        itd_sum = np.sum(itd_abs)
        delay[num,0] =i
        delay[num,1] = itd_sum
        num+= 1
        
    except FileNotFoundError:
        continue
   
energy = np.asarray(sorted(energy, key=itemgetter(1), reverse=True))
delay = np.asarray(sorted(delay, key=itemgetter(1), reverse=True))


#import matplotlib.pyplot as plt
#plt.plot(hrir_l[14,0:200])
#plt.plot(hrir_r[14,0:200])

#HRTF_data = loadmat('./KEMAR_MED2_2017_DEC-22.mat')
#azimuths=HRTF_data['azimuth_array']
#elevation=HRTF_data['elevation_array']
#HRIR_data=HRTF_data['rawData'][:,:,36:36+12]
#HRIR_data=np.reshape(HRIR_data,(2,12,1024))
#Delay_data=HRTF_data['tdData'][0,36:36+12]

