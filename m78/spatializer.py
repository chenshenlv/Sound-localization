#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Fri Jun 21 22:35:41 2019

@author: jason
"""
from help import help_fun
import math
import numpy as np
#from astropy.convolution import convolve, convolve_fft
#import random

def spatialization(azimuths,HRTF_data,sound_data,azimuths_body):
#    for i in range(1,np.size(azimuths)): 
#    aIndex=routine
#    aIndex=2
#    eIndex=8
    hp_fun=help_fun()
    left=hp_fun.HRTF_Interpolation(azimuths, azimuths_body, HRTF_data)[0] # 200*1
    right=hp_fun.HRTF_Interpolation(azimuths, azimuths_body, HRTF_data)[1] # 200*1
#    left = np.squeeze(HRTF_data['hrir_l'][aIndex, eIndex, :])  # 200*1
#    right = np.squeeze(HRTF_data['hrir_r'][aIndex, eIndex, :])  # 200*1
#    delay = hp_fun.HRTF_Interpolation(azimuths, azimuths_body, HRTF_data)[2]  # float
#    hrir_zeros = np.zeros(math.floor(abs(delay)))
#
#    if azimuths_body < 0:
#        left = np.append(left, hrir_zeros)
#        right = np.append(hrir_zeros, right)
#    else:
#        left = np.append(hrir_zeros, left)
#        right = np.append(right, hrir_zeros)
        
    data_left = np.convolve(sound_data, left, mode='valid')
    data_right = np.convolve(sound_data, right, mode='valid')
    buffer = np.vstack((data_left,data_right))
    buffer = np.transpose(buffer)
#        print('buffer size',len(buffer))
    return buffer

#x=np.array([1,2,3,4,5,6,7,8,9])
#kernal=np.array([1,2,3])
#con_full= np.convolve(x, kernal, mode='full')
#con_valid= np.convolve(x, kernal, mode='valid')
#con_same= np.convolve(x, kernal, mode='same')
#con_c_valid= np.convolve(x[0:4],kernal,mode='valid')
# moving step of sound data for next buffer has to be the the padding size not the whole frame size 