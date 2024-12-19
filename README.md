# IMSC <br/>
Sensor Interface for Soft Detectors in Surgical Simulators <br/>
Thomas Thurner, 2023, thomas.thurner22@gmail.com <br/>
Publication link:<br/>
https://ieeexplore.ieee.org/abstract/document/10124724 <br/>
<br/>
Disclaimer: <br/>
The content of the data and files is provided “as is” without any warranty whatsoever, including the accuracy or comprehensiveness. Copyright holder of this document may change the contents of this document at any time without prior notice, and copyright holder disclaims any liability in relation to recipient’s use of this document.
The open source software in this product is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. <br/>
<br/>
software license: MIT <br/>
software (C++/QT) license: GNU General Public License <br/>
hardware license: CERN OHL v2 Permissive <br/>
 <br/>
 <br/>
Content: <br/>
 <br/>
- CAD: <br/>
  - SIC_Housing.step --> Housing of the Sensory Interface Circuit <br/>
  
 <br/>
- Circuits: <br/>
  - All Files: KiCad 6.0 Project files (https://www.kicad.org/) <br/>
 <br/>
-Firmware: <br/>
  - EIT_Arduino_V3 ......................... Folder including all firmware files (two Projects: ArduinoCore, EIT_Arduino_V3), via Microchip Studio 7.0.2542 (https://www.microchip.com/en-us/tools-resources/develop/microchip-studio) <br/>
    - EIT_Arduino_V3.atsln ................... Atmel Studio Solution File <br/>
    - upload_to_proMicro_to_be_edited.bat	... File to upload resulting hex-file to microcontroller (to be edited: "Absolute_Path_To...\" needs to be replaced with the Path to the corresponding files) <br/>
 <br/>
- Software: <br/>
-- Matlab_interface: <br/>
--- executeIMSC_basic.m .................. Example code for running the SIC <br/>
--- IMSC.m ............................... Main file of SIC-class (working name is IMSC (Impedance Multi Sensing Circuit)) <br/>
--- imscData.m ........................... Data structure class of SIC <br/>
--- IDSerialComs.m ....................... Get list of COM-devices <br/>
-- Python_Interface: <br/>
--- executeIMSC_basic.py ................. Example code for running the SIC <br/>
--- executeIMSC_eit.py ................... Example code for preparing the SIC for EIT <br/>
--- ImpedanceMultiSensingCircuit.py ...... Main file of SIC-class (working name is IMSC (Impedance Multi Sensing Circuit)) <br/>
-- C++QT_Interface: <br/>
--- impedancemultisensingcircuit.cpp ..... cpp file
--- impedancemultisensingcircuit.h ....... header file
 <br/>
- Leakage_Correction_EIT.pdf ............... Comparisons of the application of compensation strategy (equation 1) to uncompensated calculations by means of an EIT at different frequencies <br/>
 <br/>
