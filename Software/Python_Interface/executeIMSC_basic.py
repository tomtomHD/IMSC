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

#0.09615685466917619-0.2102439398943835j
#0.08917880480328208-0.21170799721645087j
#0.08729820248957713-0.21259879864600076j
#0.08705568585419003-0.21331048845676595j

# Create Object / Instance of ImpedanceMultiSensingCircuit:
sic = IMSC()
# Connect to Hardware:
sic.begin()
# Print Calibration Data:
print(sic.gainU)
print(sic.gainZ)
print(sic.compensation_V_Re)
print(sic.compensation_V_Im)
print(sic.compensation_Z_Im)
print(sic.compensation_Z_Re)
print(sic.shunt_Impedance_Re)
print(sic.shunt_Impedance_Im)
print(sic.offsetAngleComplexImpedance)
print(sic.offsetAngleComplexVoltage)
# Start of Measurement-oscillation:
sic.manual_Start_MeasurementOscillation()
# Some Measurements:
# sic.measureImpedance(0, 1)
# sic.measureImpedance(14, 15)
# sic.measureVoltage(15, 14, 16)
# sic.measureVoltage(15, 14, 13)
# sic.measureVoltage(8, 9, 16)
# sic.measureVoltage(0, 1, 4)
# sic.measureVoltage(0, 1, 3)
# sic.measureVoltage(0, 1, 2)
# sic.measureVoltage(0, -1, 1)
# print("--------------------")
# print(sic.measureImpedance(0,0))
# print(sic.measureImpedance(0,1))
# print(sic.measureImpedance(0,2))
# print(sic.measureImpedance(0,-1))
# print(sic.measureImpedance(0,1))
# print(sic.measureImpedance(0,2))
print("---------SHUNT-Voltage-----------")
print(sic.measureVoltage(0, 0, 0), end = '\t')
print(sic.measureVoltage(0, 0, 0, 'complex'))
print("-----------SHUNT-Impedance---------")
print(sic.measureImpedance(0,0), end = '\t')
print(sic.measureImpedance(0,0, 'complex'))
print(sic.measureVoltage(0, -1, 0), end = '\t')
print(sic.measureVoltage(0, -1, 0, 'complex'))
print("--------------------")
a = 0
b = 3
print(sic.measureVoltage(a, b, 0), end = '\t')
print(sic.measureVoltage(a, b, 0, 'complex'))
print(sic.measureVoltage(a, b, 1), end = '\t')
print(sic.measureVoltage(a, b, 1, 'complex'))
print(sic.measureVoltage(a, b, 2), end = '\t')
print(sic.measureVoltage(a, b, 2, 'complex'))
print(sic.measureVoltage(a, b, 3), end = '\t')
print(sic.measureVoltage(a, b, 3, 'complex'))
print(sic.measureVoltage(a, b, 4), end = '\t')
print(sic.measureVoltage(a, b, 4, 'complex'))
print(sic.measureVoltage(a, b, 5), end = '\t')
print(sic.measureVoltage(a, b, 5, 'complex'))
print("--------------------")
print(sic.measureImpedance(0,1), end = '\t')
print(sic.measureImpedance(0,1, 'complex'))
print("--------------------")
# print(sic.measureVoltage(-1, -1, -1))
# print(sic.measureVoltage(0, -1, 0))
# print(sic.measureVoltage(0, -1, 1))
# print(sic.measureVoltage(0, -1, 2))
# print(sic.measureVoltage(0, -1, 3))
# print("--------------------")
# print(sic.measureVoltage(-1, -1, -1, 'complex'))
# print(sic.measureVoltage(0, 1, 1, 'complex'))
# print("--------------------")
# print(sic.measureVoltage(0, -1, 0, 'complex'))
# print(sic.measureVoltage(0, -1, 1, 'complex'))
# print(sic.measureVoltage(0, -1, 2, 'complex'))
# print(sic.measureVoltage(0, -1, 3, 'complex'))
# print(sic.measureVoltage(0, 1, 1, 'complex'))
# print("--------------------")
# print(sic.measureVoltage(0, 1, 16))
# print("--------------------")
print(sic.data.impedances)
print(sic.data.voltages)

t = time.time()
for i in range(10):
    sic.measureVoltage(a, b, 0, 'complex')
elapsed = time.time() - t
print("elapsed time for single measurement:")
print(elapsed/10)

print("############################")
print(sic.measureVoltage(4,3,2, 'complex'))
sic.setFrequency(99);
sic.manual_Start_MeasurementOscillation()
#time.sleep(3)
print(sic.measureVoltage(4,3,2, 'complex'))

sic.gpioMode(1,[1,1,1,0,0,0])

# Stop Measurement:
sic.manual_Stop_MeasurementOscillation()
# Quit:
sic.end()
