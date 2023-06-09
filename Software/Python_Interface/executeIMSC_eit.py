#!/usr/bin/envpython3
#-*-coding:utf-8-*-
"""
CreatedonTueAug211:06:232022

@author:ressl
"""

import numpy as np
import time
import serial
import serial.tools.list_ports as port_list
from ImpedanceMultiSensingCircuit import *



sic=IMSC();
sic.begin();

ex_mat=np.array([[0,1],[1,2],[2,3],[3,4],[4,5],[5,6],[6,7],[7,8],[8,9],[9,10],[10,11],[11,12],[12,13],[13,14],[14,15],[15,0]])

meas_mat=np.array([[[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14]],

[[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[10,9],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[11,10],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[12,11],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[13,12],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[14,13],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[15,14],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[0,15]],

[[1,0],[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12]],

[[2,1],[3,2],[4,3],[5,4],[6,5],[7,6],[8,7],[9,8],[10,9],[11,10],[12,11],[13,12],[14,13]]])


# Convert ex_mat to matrix format of sic.stimPattern:
sic.stimPattern = np.zeros([len(ex_mat),len(ex_mat)])
currentValue = 10#mA
for i in range(len(ex_mat)):
    sic.stimPattern[i][int(ex_mat[i][0])] =  currentValue
    sic.stimPattern[i][int(ex_mat[i][1])] = -currentValue
    

# Convert meas_mat to matrix format of sic.measPattern:
for i in range(meas_mat.shape[0]):
    sic.measPattern[i] = np.zeros([meas_mat.shape[1], meas_mat.shape[0]])
    for j in range(meas_mat.shape[1]):
        sic.measPattern[i][j][int(meas_mat[i][j][0])] =  1
        sic.measPattern[i][j][int(meas_mat[i][j][1])] = -1
        
# Write matrices to hardware:
sic.sendStimPatternToHardware()
sic.sendMeasPatternToHardware()

# Get EIT-Data:
v = sic.measureEIT()

# Here, v can be used in pyEIT!
# TODO: use PyEIT


# Quit:
sic.end()

