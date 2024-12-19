# IMSC
*Sensor Interface for Soft Detectors in Surgical Simulators*

Thomas Thurner, 2023, <br/>
[thomas.thurner22@gmail.com](mailto:thomas.thurner22@gmail.com) <br/>
 <br/>
[Publication link](https://ieeexplore.ieee.org/abstract/document/10124724)
<br/>
<br/>
**Disclaimer**: <br/>
> The content of the data and files is provided “as is” without any warranty whatsoever, including the accuracy or comprehensiveness. Copyright holder of this document may change the contents of this document at any time without prior notice, and copyright holder disclaims any liability in relation to recipient’s use of this document.
The open source software in this product is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. <br/>

## Software license: MIT
software (C++/QT) license: GNU General Public License <br/>
hardware license: CERN OHL v2 Permissive <br/>
 <br/>
 <br/>
 
## Content:
1. CAD:
   - _SIC_Housing.step_ --> Housing of the Sensory Interface Circuit

3. Circuits: 
   - All Files: KiCad 6.0 Project files (https://www.kicad.org/)

4. Firmware: 
   - EIT_Arduino_V3 --> Folder including all firmware files (two Projects: ArduinoCore, EIT_Arduino_V3), via Microchip Studio 7.0.2542 (https://www.microchip.com/en-us/tools-resources/develop/microchip-studio) 
    - _EIT_Arduino_V3.atsln_ --> Atmel Studio Solution File 
   - _upload_to_proMicro_to_be_edited.bat_	--> File to upload resulting hex-file to microcontroller (to be edited: "Absolute_Path_To...\" needs to be replaced with the Path to the corresponding files) 

5. Software: 
   - Matlab_interface: 
     - _executeIMSC_basic.m_ --> Example code for running the SIC 
     - _IMSC.m_ --> Main file of SIC-class (working name is IMSC (Impedance Multi Sensing Circuit)) 
     - _imscData.m_ --> Data structure class of SIC 
     - _IDSerialComs.m_ --> Get list of COM-devices
   - Python_Interface: 
     - _executeIMSC_basic.py_ --> Example code for running the SIC
     - _executeIMSC_eit.py_ --> Example code for preparing the SIC for EIT
     - _ImpedanceMultiSensingCircuit.py_ --> Main file of SIC-class (working name is IMSC (Impedance Multi Sensing Circuit)) 
   - C++QT_Interface:
     - _impedancemultisensingcircuit.cpp_ --> cpp file
     - _impedancemultisensingcircuit.h_ --> header file

6. _Leakage_Correction_EIT.pdf_ --> Comparisons of the application of compensation strategy (equation 1) to uncompensated calculations by means of an EIT at different frequencies <br/>
 <br/>
