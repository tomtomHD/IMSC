# IMSC
Sensor Interface for Soft Detectors in Surgical Simulators
Thomas Thurner, 2023, thomas.thurner22@gmail.com
Publication link:
https://ieeexplore.ieee.org/abstract/document/10124724

Disclaimer:
The content of the data and files is provided “as is” without any warranty whatsoever, including the accuracy or comprehensiveness. Copyright holder of this document may change the contents of this document at any time without prior notice, and copyright holder disclaims any liability in relation to recipient’s use of this document.
The open source software in this product is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 

software license: MIT
hardware license: CERN OHL v2 Permissive


Content:

-CAD:
---SIC_Housing.step ....................... Housing of the Sensory Interface Circuit

-Circuits:
---All Files: KiCad 6.0 Project files (https://www.kicad.org/)

-Firmware:
---EIT_Arduino_V3 ......................... Folder including all firmware files (two Projects: ArduinoCore, EIT_Arduino_V3), via Microchip Studio 7.0.2542 (https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)
---EIT_Arduino_V3.atsln ................... Atmel Studio Solution File
---upload_to_proMicro_to_be_edited.bat	... File to upload resulting hex-file to microcontroller (to be edited: "Absolute_Path_To...\" needs to be replaced with the Path to the corresponding files)

-Software:
---Matlab_interface:
-----executeIMSC_basic.m .................. Example code for running the SIC
-----IMSC.m ............................... Main file of SIC-class (working name is IMSC (Impedance Multi Sensing Circuit))
-----imscData.m ........................... Data structure class of SIC
-----IDSerialComs.m ....................... Get list of COM-devices
---Python_Interface:
-----executeIMSC_basic.py ................. Example code for running the SIC
-----executeIMSC_eit.py ................... Example code for preparing the SIC for EIT
-----ImpedanceMultiSensingCircuit.py ...... Main file of SIC-class (working name is IMSC (Impedance Multi Sensing Circuit))

-Leakage_Correction_EIT.pdf ............... Comparisons of the application of compensation strategy (equation 1) to uncompensated calculations by means of an EIT at different frequencies
