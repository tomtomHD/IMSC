close all;
clear all;
clc;

% Create Object / Instance of ImpedanceMultiSensingCircuit:
imsc = IMSC();%
% Connect to Hardware:
imsc.begin();
% disp Calibration Data:
disp(imsc.gainU);
disp(imsc.gainZ);
disp(imsc.compensation_V_Re);
disp(imsc.compensation_V_Im);
disp(imsc.compensation_Z_Im);
disp(imsc.compensation_Z_Re);
disp(imsc.shunt_Impedance_Re);
disp(imsc.shunt_Impedance_Im);
disp(imsc.offsetAngleComplexImpedance);
disp(imsc.offsetAngleComplexVoltage);
% Start of Measurement-oscillation:
imsc.manual_Start_MeasurementOscillation();
% Some Measurements:
% imsc.measureImpedance(0, 1)
% imsc.measureImpedance(14, 15)
% imsc.measureVoltage(15, 14, 16)
% imsc.measureVoltage(15, 14, 13)
% imsc.measureVoltage(8, 9, 16)
% imsc.measureVoltage(0, 1, 4)
% imsc.measureVoltage(0, 1, 3)
% imsc.measureVoltage(0, 1, 2)
% imsc.measureVoltage(0, -1, 1)
% disp("--------------------")
% disp(imsc.measureImpedance(0,0))
% disp(imsc.measureImpedance(0,1))
% disp(imsc.measureImpedance(0,2))
% disp(imsc.measureImpedance(0,-1))
% disp(imsc.measureImpedance(0,1))
% disp(imsc.measureImpedance(0,2))
disp("---------SHUNT-Voltage-----------");
disp(imsc.measureVoltage(0, 0, 0, 'magnitude'))
disp(imsc.measureVoltage(0, 0, 0, 'complex'))
disp("-----------SHUNT-Impedance---------")
disp(imsc.measureImpedance(0,0, 'magnitude'))
disp(imsc.measureImpedance(0,0, 'complex'))
disp(imsc.measureVoltage(0, -1, 0, 'magnitude'))
disp(imsc.measureVoltage(0, -1, 0, 'complex'))
disp("--------------------")
a = 0;
b = 3;
disp(imsc.measureVoltage(a, b, 0, 'magnitude'))
disp(imsc.measureVoltage(a, b, 0, 'complex'))
disp(imsc.measureVoltage(a, b, 1, 'magnitude'))
disp(imsc.measureVoltage(a, b, 1, 'complex'))
disp(imsc.measureVoltage(a, b, 2, 'magnitude'))
disp(imsc.measureVoltage(a, b, 2, 'complex'))
disp(imsc.measureVoltage(a, b, 3, 'magnitude'))
disp(imsc.measureVoltage(a, b, 3, 'complex'))
disp(imsc.measureVoltage(a, b, 4, 'magnitude'))
disp(imsc.measureVoltage(a, b, 4, 'complex'))
disp(imsc.measureVoltage(a, b, 5, 'magnitude'))
disp(imsc.measureVoltage(a, b, 5, 'complex'))
disp("--------------------")
disp(imsc.measureImpedance(0,1, 'magnitude'))
disp(imsc.measureImpedance(0,1, 'complex'))
disp("--------------------")
% disp(imsc.measureVoltage(-1, -1, -1))
% disp(imsc.measureVoltage(0, -1, 0))
% disp(imsc.measureVoltage(0, -1, 1))
% disp(imsc.measureVoltage(0, -1, 2))
% disp(imsc.measureVoltage(0, -1, 3))
% disp("--------------------")
% disp(imsc.measureVoltage(-1, -1, -1, 'complex'))
% disp(imsc.measureVoltage(0, 1, 1, 'complex'))
% disp("--------------------")
% disp(imsc.measureVoltage(0, -1, 0, 'complex'))
% disp(imsc.measureVoltage(0, -1, 1, 'complex'))
% disp(imsc.measureVoltage(0, -1, 2, 'complex'))
% disp(imsc.measureVoltage(0, -1, 3, 'complex'))
% disp(imsc.measureVoltage(0, 1, 1, 'complex'))
% disp("--------------------")
% disp(imsc.measureVoltage(0, 1, 16))
% disp("--------------------")


tic
    imsc.measureVoltage(0, 1, 4, 'complex')
toc
tic
    imsc.measureVoltage(0, 1, 4, 'complex')
toc
tic
    imsc.measureVoltage(0, 1, 4, 'complex')
toc
tic
    imsc.measureVoltage(0, 1, 4, 'complex')
toc
tic
    imsc.measureVoltage(0, 1, 4, 'complex')
toc

tic
for i = 1:10
    imsc.measureVoltage(0, 1, 4, 'complex')
end
toc



tic
for i = 1:10
    imsc.measureVoltage(a, b, 3, 'magnitude');
end
toc


tic
for i = 1:10
    imsc.measureImpedance(0,1, 'complex');
end
toc

tic
for i = 1:10
    imsc.measureImpedance(0,1, 'magnitude');
end
toc

% Stop Measurement:
imsc.manual_Stop_MeasurementOscillation()
% Quit:
imsc.end_()
