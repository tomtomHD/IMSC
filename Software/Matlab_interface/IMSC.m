classdef IMSC < handle
    %IMSC Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        %IMSC Construct an instance of this class
        % COM-Port Stuff:
        port = "";
        serialPort = NaN;
        % Needle Electrode:
        needleElectrode = 16;
        % Electrode and Needle Numbers:
        electrodeNumber = 16
        needleNumber = 1
        % Calibration data:
        calibData = NaN;
        % Data Object:
        data = imscData();
        % Data acquisition Finished Flag:
        finishedDataAcquisition = false;
        % Calibration Data:
        gainU = NaN;
        gainZ = NaN;
        compensation_V_Re = NaN;
        compensation_V_Im = NaN;
        compensation_Z_Im = NaN;
        compensation_Z_Re = NaN;
        shunt_Impedance_Re = NaN;
        shunt_Impedance_Im = NaN;
        % Shunt Resistance:
        Z_shunt = NaN;
        % Rotation offset of Impedances:
        offsetAngleComplexImpedance = 0;
        % Rotation offset of Voltages:
        offsetAngleComplexVoltage = 0;
        % Stimulation Pattern for EIT:
        stimPattern = zeros(16,16);
        % Measurement Pattern for EIT:
        measPattern = zeros(16,13,16);
        
    
    end
    
    methods
        function self = IMSC()
            % Constructor
            self.stimPattern =          [[-10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10, 10];
                                        [10, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -10]];
            measPattern1 =             [[0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1]];
            measPattern2 =             [[0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern3 =             [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
             measPattern4 =            [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern5 =             [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern6 =             [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern7 =             [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern8 =             [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern9 =             [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern10 =            [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern11 =            [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern12 =            [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern13 =            [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern14 =            [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [-1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1]];
            measPattern15 =            [[1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0]];
            measPattern16 =            [[0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0, 0];
                                        [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 0]];
            self.measPattern(1,:,:) = measPattern1;
            self.measPattern(2,:,:) = measPattern2;
            self.measPattern(3,:,:) = measPattern3;
            self.measPattern(4,:,:) = measPattern4;
            self.measPattern(5,:,:) = measPattern5;
            self.measPattern(6,:,:) = measPattern6;
            self.measPattern(7,:,:) = measPattern7;
            self.measPattern(8,:,:) = measPattern8;
            self.measPattern(9,:,:) = measPattern9;
            self.measPattern(10,:,:) = measPattern10;
            self.measPattern(11,:,:) = measPattern11;
            self.measPattern(12,:,:) = measPattern12;
            self.measPattern(13,:,:) = measPattern13;
            self.measPattern(14,:,:) = measPattern14;
            self.measPattern(15,:,:) = measPattern15;
            self.measPattern(16,:,:) = measPattern16;
        end

        function begin(self)
%         begin()
%
%         Description
%         -----------
%         Initializes the Module. Needs to be executed before usage.
%     
%         Returns
%         -------
%         None.
            % Get correct COM-Port:
            ports = IDSerialComs();
            nComs = size(ports);
            nComs = nComs(1);
            for p = 1:nComs
                name = ports(p,1);
                name = name{1};
                if strcmp("Arduino Leonardo",name)
                    port_ = ports(p,2);
                    port_ = port_{1};
                    self.port = "COM" + num2str(port_);
                    fprintf('Port: %s\n', self.port)
                end
            end
            % Baudrate
            baudrate = 115200;
            if(self.port == "")
                % Error with COM-Port:
                error("Device not found!")
            else
                % Open Port:
                self.serialPort = serialport(self.port, baudrate);
                self.serialPort.Timeout = 60;
            end
            self.measureCalib()
        end

        function manual_Start_MeasurementOscillation(self)
%         manual_Start_MeasurementOscillation()
%
%         Description
%         -----------
%         Starts the Signal Generator of the AD5933. It is needed to measure via 
%         'measureVoltage' and 'measureImpedance'. 
%         More complex measurement routines manage this on their own.
%     
%         Returns
%         -------
%         None.
            write(self.serialPort,'B','char');
        end

        function manual_Stop_MeasurementOscillation(self)
%         manual_Stop_MeasurementOscillation()
%
%             Description
%             -----------
%             Stops the Signal Generator of the AD5933. It is needed to 
%             end measurements via 'measureVoltage' and 'measureImpedance'. 
%             More complex measurement routines manage this on their own.
%         
%             Returns
%             -------
%             None.
            write(self.serialPort,'X','char');
        end

        function sendStimPatternToHardware(self)
%         sendStimPatternToHardware()
%
%         Description
%         -----------
%         Transmit the Stimulation Pattern from 'self.stimPattern' to the 
%         Impedance-Multi-Sensing-Circuit (needed for Electrical Impedance 
%         Tomography).
% 
%         Returns
%         -------
%         None.
            stim_pattern_shape = size(self.stimPattern);
            for j = 1:stim_pattern_shape(1)
                write(self.serialPort,'S','char');
                write(self.serialPort,j-1,'uint8');  
                pause(0.01);
                write(self.serialPort,self.stimPattern(j,:),'int8');
                pause(0.01);
            end
        end

        function sendMeasPatternToHardware(self)
%         sendMeasPatternToHardware()
%
%         Description
%         -----------
%         Transmit the Measurement Pattern from 'self.measPattern' to the 
%         Impedance-Multi-Sensing-Circuit (needed for Electrical Impedance 
%         Tomography).
% 
%         Returns
%         -------
%         None.
            meas_pattern_size = size(self.measPattern);
            meas_pattern_size = meas_pattern_size(1);
            meas_pattern_inner_size = size(self.measPattern);
            meas_pattern_inner_size = meas_pattern_inner_size(2);
            for i = 1:meas_pattern_size
                write(self.serialPort,'M','char');    
                write(self.serialPort,i-1,'uint8');    
                pause(0.01);
                for j = 1:meas_pattern_inner_size
                    write(self.serialPort,reshape(self.measPattern(i,j,:), [1,length(self.measPattern(i,j,:))]),'int8');
                    pause(0.01);
                end
            end
        end

        function setFrequency(self, frequency)
%         setFrequency(frequency)
%
%         Description
%         -----------
%         Sets the measurement frequency of the hardware. 
%         Only Values between 2 and 99 are accepted.
%     
%         Returns
%         -------
%         None.
            if frequency < 2
                frequency = 2;
            end
            if frequency > 99
                frequency = 99;
            end
            write(self.serialPort,'F','char');  
            write(self.serialPort,frequency,'int8');
            self.measureCalib();
        end

        function voltageLeakageCorrection(self, onOff)
%         voltageLeakageCorrection(onOff)
%
%         Description
%         -----------
%         Enable/Disable the voltage leakage correction of the hardware. 
%
%         Parameters
%         ----------
%         onOff : TYPE
%             DESCRIPTION. 'true' activates the leakage correction, 'false'
%             deactivates it.
% 
%         Returns
%         -------
%         None.
            if onOff
                write(self.serialPort,'L','char');  
            else
                write(self.serialPort,'l','char');  
            end
        end

        function gpioMode(self, port, value)
%         gpioMode(port, value)
%
%         Description
%         -----------
%         Set Pinmode of whole GPIO-port. 0 for input, 1 for output.
%
%         Parameters
%         ----------
%         port : TYPE
%             DESCRIPTION.  Port of GPIO to set (1,2 or 3)
% 
%         value : TYPE
%             DESCRIPTION.  Vector to set Pinmode (output--> 1, input-->0).
%                           e.g. 
%                           gpioMode(2, [1,1,0,0,0]) sets the first two
%                           pins of GPIO 2 as output and the remaining as
%                           input.
%                           Missing values are padded with 0 (input), 
%                           unnecessary values are ignored.
%                           
%         Returns
%         -------
%         None.

            % Create single number out of Vector:
            val = 0;
            for i = 1:8 
                val = val + bitsll(value(i), i-1);
                if i >=length(value)
                    break;
                end
            end
            % Send data:
            write(self.serialPort,'%','char');  
            write(self.serialPort,'d','char'); 
            write(self.serialPort,port,'uint8');  
            write(self.serialPort,val,'uint8');  
        end

        function values = gpioRead(self, port)
%         gpioRead(port)
%
%         Description
%         -----------
%         Reads Pin-values of whole GPIO-port. 0 for LOW, 1 for HIGH.  
%
%         Parameters
%         ----------
%         port : TYPE
%             DESCRIPTION.  Port of GPIO to read (1,2 or 3)
% 
%         Returns
%         -------
%         value : TYPE
%             DESCRIPTION.  Vector of Pin values.
%                           e.g. 
%                           gpioRead(2) read the Pin values of GPIO 2.
%                           [0/1, 0/1, 0/1, 0/1, 0/1, 0/1, 0/1, 0/1]
            
            % Send data:
            write(self.serialPort,'%','char');  
            write(self.serialPort,'r','char'); 
            write(self.serialPort,port,'uint8'); 
            % Read:
            val = read(self.serialPort,1,"uint8");
            values = zeros(8,1);
            for i = 1:8
                values(i) = (bitand(val, bitsll(1, i-1))~=0);
            end
        end

        function gpioWrite(self, port, value)
%         gpioWrite(port, value)
%
%         Description
%         -----------
%         Writes to Pins of whole GPIO-port. 0 for LOW, 1 for HIGH. 
%
%         Parameters
%         ----------
%         port : TYPE
%             DESCRIPTION.  Port of GPIO to write (1,2 or 3)
%
%         value : TYPE
%             DESCRIPTION.  Vector to be written (HIGH--> 1, LOW-->0).
%                           e.g. 
%                           gpioWrite(2, [1,1,0,0,0]) sets the first two
%                           pins of GPIO 2 to HIGH and the remaining to
%                           LOW.
%                           Missing values are padded with 0 (LOW), 
%                           unnecessary values are ignored.
% 
%         Returns
%         -------
%         None.

            % Create single number out of Vector:
            val = 0;
            for i = 1:8 
                val = val + bitsll(value(i), i-1);
                if i >=length(value)
                    break;
                end
            end
            % Send data:
            write(self.serialPort,'%','char');  
            write(self.serialPort,'w','char'); 
            write(self.serialPort,port,'uint8');  
            write(self.serialPort,val,'uint8');  
        end

        function measureCalib(self)
%         measureCalib()
%
%         Description
%         -----------
%         Reads out the calibration data of Impedance-Multi-Sensing-Circuit. 
%         The result gets stored in the IMSC-objects calibration-values.
%         ('gainU', 'gainZ', 'compensation_V_Re','compensation_V_Im',
%         'compensation_Z_Re','compensation_Z_Im','shunt_Impedance_Re',
%         'shunt_Impedance_Im')
% 
%         Returns
%         -------
%         None.
            % Clear old data:
            self.data = imscData();
            % Command to start measuring calibration data:
            write(self.serialPort,'K','char');
            % Store received data until finish-flag arrives:
            self.finishedDataAcquisition = false;
            while self.finishedDataAcquisition == false
                if self.serialPort.NumBytesAvailable
                    self.readDataPoint();
                end
            end
            self.gainU = self.data.other(1).magnitude;
            self.gainZ = self.data.other(2).magnitude;
            self.compensation_V_Re =  self.data.other(3).realValue;
            self.compensation_V_Im =  self.data.other(3).imagValue;
            self.compensation_Z_Re =  self.data.other(4).realValue;
            self.compensation_Z_Im =  self.data.other(4).imagValue;
            self.shunt_Impedance_Re = self.data.other(5).realValue;
            self.shunt_Impedance_Im = self.data.other(5).imagValue;
            % Shunt Resistance:
            self.Z_shunt = self.shunt_Impedance_Re + self.shunt_Impedance_Im*1i;
            % Rotation offset of Impedance Measurements:
            % Rotation offset of Impedances:
            self.offsetAngleComplexImpedance = angle(self.Z_shunt);
            % Rotation offset of Voltages:
            self.manual_Start_MeasurementOscillation();
            pause(0.01);
            self.offsetAngleComplexVoltage = 0; % In case of multiple calibrations, angle has to be reseted
            u0 = self.measureVoltage(0, -1, 0, 'complex');
            self.manual_Stop_MeasurementOscillation();
            self.offsetAngleComplexVoltage = angle(u0);
        end

        function measureAll(self, measureMode)
%         measureAll(measureMode)
%
%         Description
%         -----------
%         Measures most possible Voltages and Impedances between the Electrodes 
%         and Needle of Impedance-Multi-Sensing-Circuit. The result gets stored 
%         in the IMSC-objects data-object (self.data).
% 
%         Parameters
%         ----------
%         measureMode : TYPE, optional
%             DESCRIPTION. The default is 'magnitude'. optional, by using 'complex',
%             the measured values get stored as complex values with real and imaginary part.
% 
%         Returns
%         -------
%         None.
            if nargin == 1
                measureMode = "magnitude"; % Default Value
            end
            % Clear old data:
            self.data = imscData();
            % Command to start measuring all available data:
            if measureMode == "complex"
                write(self.serialPort,'*','char');
            else % "magnitude"
                write(self.serialPort,'+','char');
            end
            % Store received data until finish-flag arrives:
            self.finishedDataAcquisition = false;
            while self.finishedDataAcquisition == false
                if self.serialPort.NumBytesAvailable
                    self.readDataPoint()
                end
            end
        end
        
        function v = measureEIT(self, measureMode)
%         v = measureEIT(measureMode)
%
%         Description
%         -----------
%         Measures EIT Differential Voltage between the Electrodes 
%         of Impedance-Multi-Sensing-Circuit. The result gets stored 
%         in the IMSC-objects data-object (self.data). 
%         'measureElectrode' cointains both measuring electrodes 
%         (lower 4 bit: to[-]; higher 4 bit:from[+]).
%         
%         Parameters
%         ----------
%         measureMode : TYPE, optional
%             DESCRIPTION. The default is 'raw'. Optional, by using 'corrected',
%             the measured values are improved by leakage correction.
% 
% 
%         Returns
%         -------
%         v :
%             DESCRIPTION. Array (np.array) of measured voltages to be used 
%             for further calculations in e.g. pyeit.
            if nargin == 1
                measureMode = "raw"; % Default Value
            end
            % Clear old data:
            self.data = imscData();
            v = []; % Empty list for result
            % Command to start measuring all available data:
            if measureMode == "raw"
                write(self.serialPort,'c','char');
            else
                write(self.serialPort,'C','char');
            end
            % Store received data until finish-flag arrives:
            self.finishedDataAcquisition = false;
            s = size(self.measPattern);
            v = zeros(s(1)*s(2),1);
            i = 1;
            while self.finishedDataAcquisition == false
                if self.serialPort.NumBytesAvailable
                    self.readDataPoint()
                    if ~self.finishedDataAcquisition
                        v(i) = (self.data.eit(end).magnitude);
                        i = i + 1;
                    end
                end
            end
            % return v;
        end

        function volt = measureVoltage(self, fromEl, toEl, measEl, measureMode)
%         volt = measureVoltage(fromEl, toEl, measEl, measureMode)
%
%         Description
%         -----------
%         Get a voltage measurement at electrode "measEl", 
%         signal source is electrode "fromEl", signal sink is electrode "toEl".
%         If "measureMode" is set to 'magnitude' or not set, only the magnitude 
%         of the voltage is returned, otherwise a complex voltage is returned.
% 
%         Returns
%         -------
%         None.
            if nargin == 4
                measureMode = "magnitude"; % Default Value
            end
            % Polling Data:
            if measureMode == "magnitude"
                self.pollMeasurement(1, fromEl, toEl, measEl)
            else %(complex)
                self.pollMeasurement(2, fromEl, toEl, measEl)
            end
            % Reading Result
            self.readDataPoint()
            % Return Magnitude:
            if measureMode == "magnitude"
                volt = self.data.voltages(end).magnitude;
            else  %(complex)
                volt = (self.data.voltages(end).realValue) + (self.data.voltages(end).imagValue)*1i;
            end
            % return volt;
        end

        function imp = measureImpedance(self, fromEl, toEl, measureMode)
%         imp = measureImpedance(fromEl, toEl, measureMode)
%
%         Description
%         -----------
%         Get an impedance measurement between electrode "fromEl" and 
%         electrode "toEl". If "measureMode" is set to 'magnitude' or not set, 
%         only the magnitude of the impedance is returned, otherwise a 
%         complex impedance is returned.
% 
%         Returns
%         -------
%         None.
            if nargin == 3
                measureMode = "magnitude"; % Default Value
            end
            % Polling Data:
            if measureMode == "magnitude"
                self.pollMeasurement(3, fromEl, toEl)
            else %(complex)
                self.pollMeasurement(4, fromEl, toEl)
            end
            % Reading Result
            self.readDataPoint()
            % Return Magnitude:
            if measureMode == "magnitude"
                imp = self.data.impedances(end).magnitude;
            else %(complex)
                imp = (self.data.impedances(end).realValue) + (self.data.impedances(end).imagValue)*1i;
            end
        end

        function pollMeasurement(self, mode, fromEl, toEl, measEl)
%         pollMeasurement(mode, fromEl, toEl, measEl)
%
%         Description
%         -----------
%         Requests a measurement from the hardware. Should only be used internally. 
% 
%         Returns
%         -------
%         None.
            if nargin == 4
                measEl = -1;    % Default Value
            end
            write(self.serialPort,'@','char');
            write(self.serialPort,mode,'int8');
            write(self.serialPort,fromEl,'int8');
            write(self.serialPort,toEl,'int8');
            write(self.serialPort,measEl,'int8');
        end

        function v = byte4float(self, c)
%         v = byte4float(c)
%
%         Description
%         -----------
%         Converts four Bytes of Data into a float-number (IEEE 754)
% 
%         Parameters
%         ----------
%         c : TYPE
%             DESCRIPTION. List of four Byte.
% 
%         Returns
%         -------
%         TYPE
%             DESCRIPTION. Float number, calculated out of Byte-list c

            E = bitshift(bitand(c(4), 127),1) + bitshift(bitand(c(3), 128),-7);
            m = c(1) + bitshift(c(2),8) + bitshift(bitand(c(3), 127),16);
            v = (1+(2.0^(-23))*m) * (2.0^(E-127));
            if c(4) > 127
                v=-v;
            end
        end

        function readDataPoint(self)
%         readDataPoint()
%
%         Description
%         -----------
%         Reads single data-transmission from COM-port and stores is into the
%         lists of the classes data-object (self.data). The classification of
%         whether the data is voltage, impedance or other measured value is made
%         automatically on the basis of the data. The data is added to the 
%         corresponding list. 
%         The classification is done by a data type byte that is sent along 
%         with the data. A type for ending long transmission sequences is also 
%         included.
%         This command does not include the request for the data itself. 
%         This must be done externally beforehand.
% 
%         Returns
%         -------
%         None.
            % Read from COM-Port:
            sDataBytes = read(self.serialPort,1,"uint8");
            % Extract Mode:
                % Mode0-->Data acquisition finished or ERROR,
                % Mode1-->Magnitude of Voltage, 
                % Mode2-->Complex Voltage,
                % Mode3-->Magnitude of Impedance, 
                % Mode4-->Complex Impedance,
                % Mode5-->Magnitude of other Data,
                % Mode6-->Complex other Data,
                % Mode7-->EIT Data (Magnitude)
            mode = sDataBytes(1);
            % Read from COM-Port:
            if mode == 0 % QUIT
                self.finishedDataAcquisition = true;
                return
            elseif mode >= 1 && mode <= 7
                if mod(mode,2) == 0% Complex Data
                    sDataBytes = read(self.serialPort,11,"uint8");
                else % Magnitude Data
                    sDataBytes = read(self.serialPort,7,"uint8");
                end
                % Extract and store Data:
                if (sDataBytes(1) >= 0 && sDataBytes(1) <= self.electrodeNumber+self.needleNumber)
                    fromEl = sDataBytes(1);
                else 
                    fromEl = -1;
                end
                if (sDataBytes(2) >= 0 && sDataBytes(2) <= self.electrodeNumber+self.needleNumber) 
                    toEl = sDataBytes(2);
                else 
                    toEl = -1;
                end
                if ((sDataBytes(3) >= 0 && sDataBytes(3) <= self.electrodeNumber+self.needleNumber) || mode == 7)
                    measureEl = sDataBytes(3);
                else
                    measureEl = -1;
                end
                if mod(mode, 2) == 0 % Complex Data
                    floatValue1 = self.byte4float([sDataBytes(4), sDataBytes(5), sDataBytes(6), sDataBytes(7)]);
                    floatValue2 = self.byte4float([sDataBytes(8), sDataBytes(9), sDataBytes(10), sDataBytes(11)]);
                    % Choose Modes:
                    if mode == 2 % Complex Voltage:
                        % Correct Phase shift of Hardware:
                        % (negative sign before imaginary part because of Z=u/i, 
                        % sign gets switched by AD5933 because of assuming
                        % calculating Impedance out of measured Current
                        % [1/(a+bj) = (a-bj)/(a^2+b^2), sign of Imaginary part
                        % swithes while inverting term],
                        % needs to be undone here)
                        [floatValue1, floatValue2] = self.correctComplexByRotation(self.offsetAngleComplexVoltage, floatValue1, -floatValue2);
                        ds.fromElectrode = fromEl;
                        ds.toElectrode = toEl;
                        ds.measureElectrode = measureEl;
                        ds.realValue = floatValue1;
                        ds.imagValue = floatValue2;
                        ds.magnitude = NaN;
                        self.data.voltages(end+1) = (ds);
                    elseif mode == 4 % Complex Resistance:
                        % Correct Phase shift of Hardware:
                        [floatValue1, floatValue2] = self.correctComplexByRotation(self.offsetAngleComplexImpedance, floatValue1, floatValue2);
                        ds.fromElectrode = fromEl;
                        ds.toElectrode = toEl;
                        ds.measureElectrode = NaN;
                        ds.realValue = floatValue1;
                        ds.imagValue = floatValue2;
                        ds.magnitude = NaN;
                        self.data.impedances(end+1) = (ds);
                    elseif mode == 6 % Complex other Data:
                        ds.fromElectrode = fromEl;
                        ds.toElectrode = toEl;
                        ds.measureElectrode = measureEl;
                        ds.realValue = floatValue1;
                        ds.imagValue = floatValue2;
                        ds.magnitude = NaN;
                        self.data.other(end+1) = (ds);
                    end
                        
                else % Magnitude Data
                    floatValue1 = self.byte4float([sDataBytes(4), sDataBytes(5), sDataBytes(6), sDataBytes(7)]);
                    if mode == 1 % Magnitude of Voltage:
                        ds.fromElectrode = fromEl;
                        ds.toElectrode = toEl;
                        ds.measureElectrode = measureEl;
                        ds.realValue = NaN;
                        ds.imagValue = NaN;
                        ds.magnitude = floatValue1;
                        self.data.voltages(end+1) = (ds);
                    elseif mode == 3 % Magnitude of Resistance:
                        ds.fromElectrode = fromEl;
                        ds.toElectrode = toEl;
                        ds.measureElectrode = NaN;
                        ds.realValue = NaN;
                        ds.imagValue = NaN;
                        ds.magnitude = floatValue1;
                        self.data.impedances(end+1) = (ds);
                    elseif mode == 5 % Magnitude of other Data:
                        ds.fromElectrode = fromEl;
                        ds.toElectrode = toEl;
                        ds.measureElectrode = measureEl;
                        ds.realValue = NaN;
                        ds.imagValue = NaN;
                        ds.magnitude = floatValue1;
                        self.data.other(end+1) = (ds);
                    elseif mode == 7 % EIT Data (Magnitude):
                        ds.fromElectrode = fromEl;
                        ds.toElectrode = toEl;
                        ds.measureElectrode = measureEl;
                        ds.realValue = NaN;
                        ds.imagValue = NaN;
                        ds.magnitude = floatValue1;
                        self.data.eit(end+1) = (ds);
                    end
                end
            end
        end    

        function [realC, imagC] = correctComplexByRotation(self, angle_, real, imag)
%         [realC, imagC] = correctComplexByRotation(angle_, real, imag)
%
%         Description
%         -----------
%         Rotates Complex Values by the offset of the IMSC-Device
%         
%         Parameters
%         ----------
%         complexNumber : complex-float-value
%             Description. Complex Value
%         
%         Returns
%         -------
%             Description. Corrected (rotated) value.
            %return [real, imag]
            complexNumber = real + imag*1i;
            absV = abs(complexNumber);
            angleV = angle(complexNumber) - angle_;
            realC = absV*cos(angleV);
            imagC = absV*sin(angleV);
        end
    
        function end_(self)
%         end_()
%
%         Description
%         -----------
%         Terminates the communication via the COM-port and releases it again.
% 
%         Returns
%         -------
%         None.
            % Stop Signal Generator:
            self.manual_Stop_MeasurementOscillation()
            % Close Port:
            % No Solution yet...
        end

    end
end

