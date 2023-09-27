# -*- coding: utf-8 -*-
"""
Thomas Thurner
"""

import numpy as np
import time

import serial
import serial.tools.list_ports as port_list


electrodeNumber = 16
needleNumber = 1
measuredVoltagesPerStimulation = 13


class imscData:
    def __init__(self):
        # Measured Impedances:
        self.impedances = []
        # Measured Voltages:
        self.voltages   = []
        # Measured other Data (temporary):
        self.other      = []
        # Measured EIT-Data:
        self.eit        = []

class IMSC:
    def __init__(self):
        # COM-Port Stuff:
        self.port = None
        self.serialPort = None
        # Needle Electrode:
        self.needleElectrode = 16
        # Calibration data:
        self.calibData = None
        # Data Object:
        self.data = imscData()
        # Data acquisition Finished Flag:
        self.finishedDataAcquisition = False
        # Calibration Data:
        self.gainU = np.nan
        self.gainZ = np.nan
        self.compensation_V_Re = np.nan
        self.compensation_V_Im = np.nan
        self.compensation_Z_Im = np.nan
        self.compensation_Z_Re = np.nan
        self.shunt_Impedance_Re = np.nan
        self.shunt_Impedance_Im = np.nan
        # Shunt Resistance:
        self.Z_shunt = np.nan
        # Rotation offset of Impedances:
        self.offsetAngleComplexImpedance = 0
        # Rotation offset of Voltages:
        self.offsetAngleComplexVoltage = 0
        # Stimulation Pattern for EIT:
        self.stimPattern = np.array([[-10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10],
                                    [10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10]])
        # Measurement Pattern for EIT:
        self.measPattern = np.array([[[0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1]],
                                    [[0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]],
                                    [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0]],
                                    [[0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0],
                                    [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0]]]);
    
    
    def begin(self):
        ports = list(port_list.comports())
        for p in ports:
            if "Arduino Leonardo" in p.description:
                self.port = p.name
                print (p)
                
        if(self.port == None):
            raise Exception("Device not found!")
        else:
            #self.serialPort = serial.Serial('/dev/' + self.port)
            self.serialPort = serial.Serial(self.port)
        time.sleep(1)
        # Calibration:
        self.measureCalib()


    def manual_Start_MeasurementOscillation(self):
        """
    
        Description
        -----------
        Starts the Signal Generator of the AD5933. It is needed to measure via 
        'measureVoltage' and 'measureImpedance'. 
        More complex measurement routines manage this on their own.
    
        Returns
        -------
        None.
    
        """
        self.serialPort.write(bytearray('B','ascii'))
        
    def manual_Stop_MeasurementOscillation(self):
        """
    
        Description
        -----------
        Stops the Signal Generator of the AD5933. It is needed to 
        end measurements via 'measureVoltage' and 'measureImpedance'. 
        More complex measurement routines manage this on their own.
    
        Returns
        -------
        None.
    
        """
        self.serialPort.write(bytearray('X','ascii'))
        
    def sendStimPatternToHardware(self):
        """
        
        Description
        -----------
        Transmit the Stimulation Pattern from 'self.stimPattern' to the 
        Impedance-Multi-Sensing-Circuit (needed for Electrical Impedance 
        Tomography).

        Returns
        -------
        None.

        """
        stim_pattern_shape = self.stimPattern.shape  
        for j in range(0,stim_pattern_shape[0]):
            self.serialPort.write(bytearray('S','ascii'))
            self.serialPort.write(j.to_bytes(length = 1, byteorder = 'big', signed = True))   
            time.sleep(0.01)
            for k in range(0, stim_pattern_shape[1]):
                self.serialPort.write(bytearray([np.int8(self.stimPattern[j,k]).tobytes()[0]]))
            time.sleep(0.01)        
        
        
    def sendMeasPatternToHardware(self):
        """
        
        Description
        -----------
        Transmit the Measurement Pattern from 'self.measPattern' to the 
        Impedance-Multi-Sensing-Circuit (needed for Electrical Impedance 
        Tomography).

        Returns
        -------
        None.

        """
        meas_pattern_shape = self.measPattern.shape
        for j in range(0,meas_pattern_shape[0]):
            self.serialPort.write(bytearray('M','ascii'))
            self.serialPort.write(j.to_bytes(length = 1, byteorder = 'big', signed = True))   
            time.sleep(0.01)
            for k in range(0,meas_pattern_shape[1]):
                for l in range(0,meas_pattern_shape[2]):
                    self.serialPort.write(bytearray([self.measPattern[j,k,l].tobytes()[0]]))
        time.sleep(0.01)
    
    def setFrequency(self, frequency):
        """
    
        Description
        -----------
        Sets the measurement frequency of the hardware. 
        Only Values between 2 and 99 are accepted.
    
        Returns
        -------
        None.
    
        """
        if frequency < 2: frequency = 2
        if frequency > 99: frequency = 99
        self.serialPort.write(bytearray('F','ascii'))
        self.serialPort.write(bytearray([np.int8(frequency).tobytes()[0]]))
        #self.serialPort.write(bytearray(str(frequency), 'ascii'))
        
    def voltageLeakageCorrection(self, onOff):
        """
    
        Description
        -----------
        Enable/Disable the voltage leakage correction of the hardware. 
    
        Parameters
         ----------
         onOff : TYPE
             DESCRIPTION. 'true' activates the leakage correction, 'false'
             deactivates it.
             
        Returns
        -------
        None.
    
        """
        
        if onOff:
            self.serialPort.write(bytearray('L','ascii'))
        else:
            self.serialPort.write(bytearray('l','ascii'))
        #
        
    def gpioMode(self, port, value):
        """
         Description
         -----------
         Set Pinmode of whole GPIO-port. 0 for input, 1 for output.

         Parameters
         ----------
         port : TYPE
             DESCRIPTION.  Port of GPIO to set (1,2 or 3)
 
         value : TYPE
             DESCRIPTION.  List to set Pinmode (output--> 1, input-->0).
                           e.g. 
                           gpioMode(2, [1,1,0,0,0]) sets the first two
                           pins of GPIO 2 as output and the remaining as
                           input.
                           Missing values are padded with 0 (input), 
                           unnecessary values are ignored.
                           
         Returns
         -------
         None.
         """

        # Create single number out of Vector:
        val = 0
        for i in range(0,8):
            val = val + (value[i]<<i)
            if i >=len(value)-1:
                break
            #
        #
        # Send data:
        self.serialPort.write(bytearray('%','ascii'))
        self.serialPort.write(bytearray('d','ascii'))
        self.serialPort.write(int(port).to_bytes(1,byteorder='big',signed=False))
        self.serialPort.write(int(val).to_bytes(1,byteorder='big',signed=False))
        

    def gpioRead(self, port):
        """
         Description
         -----------
         Reads Pin-values of whole GPIO-port. 0 for LOW, 1 for HIGH.  

         Parameters
         ----------
         port : TYPE
             DESCRIPTION.  Port of GPIO to read (1,2 or 3)
 
         Returns
         -------
         value : TYPE
             DESCRIPTION.  List of Pin values.
                           e.g. 
                           gpioRead(2) read the Pin values of GPIO 2.
                           [0/1, 0/1, 0/1, 0/1, 0/1, 0/1, 0/1, 0/1]
        """     
        
        # Send data:
        self.serialPort.write(bytearray('%','ascii'))
        self.serialPort.write(bytearray('r','ascii'))
        self.serialPort.write(int(port).to_bytes(1,byteorder='big',signed=False))
        # Read:
        val = self.serialPort.read(1)
        val = val[0]
        values = []
        for i in range(0,8):
            values.append((val & 1<<i)!=0)
        #
        return values
    

    def gpioWrite(self, port, value):
        """
         Description
         -----------
         Writes to Pins of whole GPIO-port. 0 for LOW, 1 for HIGH. 

         Parameters
         ----------
         port : TYPE
             DESCRIPTION.  Port of GPIO to write (1,2 or 3)

         value : TYPE
             DESCRIPTION.  List to be written (HIGH--> 1, LOW-->0).
                           e.g. 
                           gpioWrite(2, [1,1,0,0,0]) sets the first two
                           pins of GPIO 2 to HIGH and the remaining to
                           LOW.
                           Missing values are padded with 0 (LOW), 
                           unnecessary values are ignored.
 
         Returns
         -------
         None.
        """
        
        # Create single number out of Vector:
        val = 0
        for i in range(0,8):
            val = val + (value[i]<<i)
            if i >=len(value)-1:
                break
            #
        #
        # Send data:
        self.serialPort.write(bytearray('%','ascii'))
        self.serialPort.write(bytearray('w','ascii'))
        self.serialPort.write(int(port).to_bytes(1,byteorder='big',signed=False))
        self.serialPort.write(int(val).to_bytes(1,byteorder='big',signed=False))
        
        
    def measureCalib(self):
        """
        
        Description
        -----------
        Reads out the calibration data of Impedance-Multi-Sensing-Circuit. 
        The result gets stored in the IMSC-objects calibration-values.
        ('gainU', 'gainZ', 'compensation_V_Re','compensation_V_Im',
        'compensation_Z_Re','compensation_Z_Im','shunt_Impedance_Re',
        'shunt_Impedance_Im')

        Returns
        -------
        None.

        """
        # Clear old data:
        self.data = imscData()
        # Command to start measuring calibration data:
        self.serialPort.write(bytearray('K','ascii'))
        # Store received data until finish-flag arrives:
        self.finishedDataAcquisition = False
        while self.finishedDataAcquisition == False:
            if self.serialPort.in_waiting:
                self.readDataPoint()
            #
        #
        self.gainU = self.data.other[0]['magnitude']
        self.gainZ = self.data.other[1]['magnitude']
        self.compensation_V_Re =  self.data.other[2]['realValue']
        self.compensation_V_Im =  self.data.other[2]['imagValue']
        self.compensation_Z_Re =  self.data.other[3]['realValue']
        self.compensation_Z_Im =  self.data.other[3]['imagValue']
        self.shunt_Impedance_Re = self.data.other[4]['realValue']
        self.shunt_Impedance_Im = self.data.other[4]['imagValue']
        # Shunt Resistance:
        self.Z_shunt = complex(self.shunt_Impedance_Re, self.shunt_Impedance_Im)
        # Rotation offset of Impedance Measurements:
        # Rotation offset of Impedances:
        self.offsetAngleComplexImpedance = np.angle(self.Z_shunt)
        # Rotation offset of Voltages:
        self.manual_Start_MeasurementOscillation()
        time.sleep(0.01)
        self.offsetAngleComplexVoltage = 0; # In case of multiple calibrations, angle has to be reseted
        u0 = self.measureVoltage(0, -1, 0, 'complex')
        self.manual_Stop_MeasurementOscillation()
        self.offsetAngleComplexVoltage = np.angle(complex(u0.real, u0.imag))
        return
        
        
    def measureAll(self, measureMode = 'magnitude'):
        """
        
        Description
        -----------
        Measures most possible Voltages and Impedances between the Electrodes 
        and Needle of Impedance-Multi-Sensing-Circuit. The result gets stored 
        in the IMSC-objects data-object (self.data).

        Parameters
        ----------
        measureMode : TYPE, optional
            DESCRIPTION. The default is 'magnitude'. optional, by using 'complex',
            the measured values get stored as complex values with real and imaginary part.

        Returns
        -------
        None.

        """
        # Clear old data:
        self.data = imscData()
        # Command to start measuring all available data:
        if measureMode == 'complex':
            self.serialPort.write(bytearray('*','ascii'))
        else:
            self.serialPort.write(bytearray('+','ascii'))
        # Store received data until finish-flag arrives:
        self.finishedDataAcquisition = False
        while self.finishedDataAcquisition == False:
            if self.serialPort.in_waiting:
                self.readDataPoint()
            #
        #
        
    def measureEIT(self, measureMode = 'raw'):
        """
        
        Description
        -----------
        Measures EIT Differential Voltage between the Electrodes 
        of Impedance-Multi-Sensing-Circuit. The result gets stored 
        in the IMSC-objects data-object (self.data). 
        'measureElectrode' cointains both measuring electrodes 
        (lower 4 bit: to[-]; higher 4 bit:from[+]).
        
        Parameters
        ----------
        measureMode : TYPE, optional
            DESCRIPTION. The default is 'raw'. Optional, by using 'corrected',
            the measured values are improved by leakage correction.


        Returns
        -------
        v :
            DESCRIPTION. Array (np.array) of measured voltages to be used 
            for further calculations in e.g. pyeit.

        """
        # Clear old data:
        self.data = imscData()
        v = [] # Empty list for result
        # Command to start measuring all available data:
        if measureMode == 'raw':
            self.serialPort.write(bytearray('c','ascii'))
        else:
            self.serialPort.write(bytearray('C','ascii'))
        # Store received data until finish-flag arrives:
        self.finishedDataAcquisition = False
        while self.finishedDataAcquisition == False:
            if self.serialPort.in_waiting:
                self.readDataPoint()
                if not self.finishedDataAcquisition:
                    v.append(self.data.eit[-1]['magnitude'])
                #
            #
        #
        return np.array(v)
    
    def measureVoltage(self, fromEl, toEl, measEl, measureMode = 'magnitude'):
        """
        
        Description
        -----------
        Get a voltage measurement at electrode "measEl", 
        signal source is electrode "fromEl", signal sink is electrode "toEl".
        If "measureMode" is set to 'magnitude' or not set, only the magnitude 
        of the voltage is returned, otherwise a complex voltage is returned.

        Returns
        -------
        None.

        """
        # Polling Data:
        if(measureMode == 'magnitude'):
            self._pollMeasurement(1, fromEl, toEl, measEl)
        else: #(complex)
            self._pollMeasurement(2, fromEl, toEl, measEl)
        # Reading Result
        self.readDataPoint()
        # Return Magnitude:
        if(measureMode == 'magnitude'):
            return self.data.voltages[-1]['magnitude']
        else: #(complex)
            return complex((self.data.voltages[-1]['realValue']), (self.data.voltages[-1]['imagValue']))
            #return np.sqrt((self.data.voltages[-1]['realValue'])**2 + (self.data.voltages[-1]['imagValue'])**2)
        
    def measureImpedance(self, fromEl, toEl, measureMode = 'magnitude'):
        """
        
        Description
        -----------
        Get an impedance measurement between electrode "fromEl" and 
        electrode "toEl". If "measureMode" is set to 'magnitude' or not set, 
        only the magnitude of the impedance is returned, otherwise a 
        complex impedance is returned.

        Returns
        -------
        None.

        """
        # Polling Data:
        if(measureMode == 'magnitude'):
            self._pollMeasurement(3, fromEl, toEl)
        else: #(complex)
            self._pollMeasurement(4, fromEl, toEl)
        # Reading Result
        self.readDataPoint()
        # Return Magnitude:
        if(measureMode == 'magnitude'):
            return self.data.impedances[-1]['magnitude']
        else: #(complex)
            return complex((self.data.impedances[-1]['realValue']),(self.data.impedances[-1]['imagValue']))
            #return np.sqrt((self.data.impedances[-1]['realValue'])**2 + (self.data.impedances[-1]['imagValue'])**2)
        
    def _pollMeasurement(self, mode, fromEl, toEl, measEl = -1):
        """
        
        Description
        -----------
        Requests a measurement from the hardware. Should only be used internally. 

        Returns
        -------
        None.

        """
        self.serialPort.write(bytearray('@','ascii'))
        self.serialPort.write(int(mode).to_bytes(1,byteorder='big',signed=True))
        self.serialPort.write(int(fromEl).to_bytes(1,byteorder='big',signed=True))
        self.serialPort.write(int(toEl).to_bytes(1,byteorder='big',signed=True))
        self.serialPort.write(int(measEl).to_bytes(1,byteorder='big',signed=True))
        
    def byte4float(self, c):
        """
        Description
        -----------
        Converts four Bytes of Data into a float-number (IEEE 754)

        Parameters
        ----------
        c : TYPE
            DESCRIPTION. List of four Byte.

        Returns
        -------
        TYPE
            DESCRIPTION. Float number, calculated out of Byte-list c

        """
        E = ((c[3] & 127)<<1) + ((c[2] & 128)>>7)
        m = c[0] + (c[1]<<8) + ((c[2]& 127)<<16)
        v = (1+pow(2.0, -23)*m) * pow(2.0, E-127)
        if c[3] > 127:
            return -v
        else:
            return v
        #
        
        
    def readDataPoint(self):
        """
        Description
        -----------
        Reads single data-transmission from COM-port and stores is into the
        lists of the classes data-object (self.data). The classification of
        whether the data is voltage, impedance or other measured value is made
        automatically on the basis of the data. The data is added to the 
        corresponding list. 
        The classification is done by a data type byte that is sent along 
        with the data. A type for ending long transmission sequences is also 
        included.
        This command does not include the request for the data itself. 
        This must be done externally beforehand.

        Returns
        -------
        None.

        """
        # Read from COM-Port:
        sDataBytes = self.serialPort.read(1)
        # Extract Mode:
            # Mode0-->Data acquisition finished or ERROR,
            # Mode1-->Magnitude of Voltage, 
            # Mode2-->Complex Voltage,
            # Mode3-->Magnitude of Impedance, 
            # Mode4-->Complex Impedance,
            # Mode5-->Magnitude of other Data,
            # Mode6-->Complex other Data,
            # Mode7-->EIT Data (Magnitude)
        mode = sDataBytes[0] 
        # Read from COM-Port:
        if mode == 0: # QUIT
            self.finishedDataAcquisition = True
            return
        elif mode >= 1 and mode <= 7:
            if mode % 2 == 0:# Complex Data
                sDataBytes = self.serialPort.read(11)
            else: # Magnitude Data
                sDataBytes = self.serialPort.read(7)
            # Extract and store Data:
            fromEl = sDataBytes[0] if (sDataBytes[0] >= 0 and sDataBytes[0] <= electrodeNumber+needleNumber) else -1
            toEl = sDataBytes[1] if (sDataBytes[1] >= 0 and sDataBytes[1] <= electrodeNumber+needleNumber) else -1
            measureEl = sDataBytes[2] if ((sDataBytes[2] >= 0 and sDataBytes[2] <= electrodeNumber+needleNumber) or mode == 7) else -1
            if mode % 2 == 0: # Complex Data
                floatValue1 = self.byte4float([sDataBytes[3], sDataBytes[4], sDataBytes[5], sDataBytes[6]])
                floatValue2 = self.byte4float([sDataBytes[7], sDataBytes[8], sDataBytes[9], sDataBytes[10]])
                # Choose Modes:
                if mode == 2: # Complex Voltage:
                    # Correct Phase shift of Hardware:
                    # (negative sign before imaginary part because of Z=u/i, 
                    # sign gets switched by AD5933 because of assuming
                    # calculating Impedance out of measured Current
                    # [1/(a+bj) = (a-bj)/(a^2+b^2), sign of Imaginary part
                    # swithes while inverting term],
                    # needs to be undone here)
                    [floatValue1, floatValue2] = self.correctComplexByRotation(self.offsetAngleComplexVoltage, floatValue1, -floatValue2)
                    self.data.voltages.append({'fromElectrode'   : fromEl,
                                               'toElectrode'     : toEl,
                                               'measureElectrode': measureEl,
                                               'realValue'       : floatValue1,
                                               'imagValue'       : floatValue2,
                                               'magnitude'       : np.nan})
                elif mode == 4: # Complex Resistance:
                    # Correct Phase shift of Hardware:
                    [floatValue1, floatValue2] = self.correctComplexByRotation(self.offsetAngleComplexImpedance, floatValue1, floatValue2)
                    self.data.impedances.append({'fromElectrode'   : fromEl,
                                                  'toElectrode'     : toEl,
                                                  'measureElectrode': np.nan,
                                                  'realValue'       : floatValue1,
                                                  'imagValue'       : floatValue2,
                                                  'magnitude'       : np.nan})
                elif mode == 6: # Complex other Data:
                    self.data.other.append({'fromElectrode'   : fromEl,
                                            'toElectrode'     : toEl,
                                            'measureElectrode': measureEl,
                                            'realValue'       : floatValue1,
                                            'imagValue'       : floatValue2,
                                            'magnitude'       : np.nan})
                    
            else: # Magnitude Data
                floatValue1 = self.byte4float([sDataBytes[3], sDataBytes[4], sDataBytes[5], sDataBytes[6]])
                if mode == 1: # Magnitude of Voltage:
                    self.data.voltages.append({'fromElectrode'   : fromEl,
                                               'toElectrode'     : toEl,
                                               'measureElectrode': measureEl,
                                               'realValue'       : np.nan,
                                               'imagValue'       : np.nan,
                                               'magnitude'       : floatValue1})
                elif mode == 3: # Magnitude of Resistance:
                    self.data.impedances.append({'fromElectrode'   : fromEl,
                                                  'toElectrode'     : toEl,
                                                  'measureElectrode': np.nan,
                                                  'realValue'       : np.nan,
                                                  'imagValue'       : np.nan,
                                                  'magnitude'       : floatValue1})
                elif mode == 5: # Magnitude of other Data:
                    self.data.other.append({'fromElectrode'   : fromEl,
                                            'toElectrode'     : toEl,
                                            'measureElectrode': measureEl,
                                            'realValue'       : np.nan,
                                            'imagValue'       : np.nan,
                                            'magnitude'       : floatValue1})
                elif mode == 7: # EIT Data (Magnitude):
                    self.data.eit.append({'fromElectrode'   : fromEl,
                                          'toElectrode'     : toEl,
                                          'measureElectrode': measureEl,
                                          'realValue'       : np.nan,
                                          'imagValue'       : np.nan,
                                          'magnitude'       : floatValue1})
                #
            #
        #
            
    def correctComplexByRotation(self, angle, real, imag):
        """
        Description
        -----------
        Rotates Complex Values by the offset of the IMSC-Device
        
        Parameters
        ----------
        complexNumber : complex-float-value
            Description. Complex Value
        
        Returns
        -------
            Description. Corrected (rotated) value.

        """
        #return [real, imag]
        complexNumber = complex(real, imag)
        absV = np.abs(complexNumber)
        angleV = np.angle(complexNumber) - angle
        return [absV*np.cos(angleV), absV*np.sin(angleV)]
        
    
    def end(self):
        """
        Description
        -----------
        Terminates the communication via the COM-port and releases it again.

        Returns
        -------
        None.

        """
        # Stop Signal Generator:
        self.manual_Stop_MeasurementOscillation()
        # Close Port:
        self.serialPort.close()
        return 
    #
#
