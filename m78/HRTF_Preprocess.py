#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Jun 26 21:52:40 2019

@author: jason
"""

from scipy.io import loadmat,savemat
import numpy as np
#
#HRTF_data = loadmat('./KEMAR_MED2_2017_DEC-22.mat')
#azimuths=np.reshape(HRTF_data['azimuth_array'],(12))
#elevation=np.reshape(HRTF_data['elevation_array'],(6))
#HRIR_data=HRTF_data['rawData'][:,:,36:36+12]
#HRIR_data=np.reshape(HRIR_data,(2,12,1024))
#hrir_l=HRIR_data[0,:,:]
#hrir_r=HRIR_data[1,:,:]
#ITD=np.reshape(HRTF_data['tdData'][0,36:36+12],(12))
#savemat('Propcessed_HRTF.mat', {'ITD':ITD,'hrir_l':hrir_l,'hrir_r':hrir_r,'azimuths':azimuths,
#                                'elevation':elevation},oned_as='column')
#test=loadmat('Propcessed_HRTF.mat')
#delay=test['ITD']

HRTF_data = loadmat('./small_pinna_final.mat')
azimuths=np.zeros((72))
for i in range(72):
    azimuths[i]=-180+i*5
#azimuths=np.reshape(HRTF_data['azimuth_array'],(12))
#elevation=np.reshape(HRTF_data['elevation_array'],(6))
#HRIR_data=HRTF_data['rawData'][:,:,36:36+12]
#HRIR_data=np.reshape(HRIR_data,(2,12,1024))
hrir_l=np.reshape(HRTF_data['left'],(72,200))
hrir_r=np.reshape(HRTF_data['right'],(72,200))
temp=np.zeros((72,200))
temp[0:36,:] = hrir_l[36:72,:]
temp[36:72,:] = hrir_l[0:36,:]
hrir_l=temp
temp=np.zeros((72,200))
temp[0:36,:] = hrir_r[36:72,:]
temp[36:72,:] = hrir_r[0:36,:]
hrir_r = temp

savemat('special_Propcessed_HRTF.mat', {'hrir_l':hrir_l,'hrir_r':hrir_r,'azimuths':azimuths,
                                },oned_as='column')
test=loadmat('special_Propcessed_HRTF.mat')


import matplotlib.pyplot as plt
plt.plot(hrir_l[35,:])
plt.plot(hrir_r[35,:])