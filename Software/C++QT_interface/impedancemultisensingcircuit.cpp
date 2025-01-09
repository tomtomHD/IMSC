#include "impedancemultisensingcircuit.h"

#include <QtSerialPort/QSerialPort>

// Circular buffer for data storage:

template <typename T>
CircularBuffer<T>::CircularBuffer(size_t capacity) : data(capacity), index(0) {}

template <typename T>
void CircularBuffer<T>::push(const T& value) {
    data[index] = value;
    index = (index + 1) % data.size();
}

template <typename T>
const T& CircularBuffer<T>::operator[](size_t i) const {
    return data[(index + i) % data.size()];
}

template <typename T>
size_t CircularBuffer<T>::size() const {
    return data.size();
}

template <typename T>
const T& CircularBuffer<T>::back() const {
    return data[(index + data.size() - 1) % data.size()];
}

template <typename T>
void CircularBuffer<T>::clear() {
    data.resize(data.capacity()); // Reallocate to original capacity
    index = 0; // Reset the index to the beginning
}



// Data class for storage:

IMSCdata::IMSCdata() :
    impedances(MEASURE_BUFFER_IMPEDANCES_LENGTH),
    voltages(MEASURE_BUFFER_VOLTAGES_LENGTH),
    other(MEASURE_BUFFER_OTHER_LENGTH),
    eit(EIT_DATA_VECTOR_LENGTH) {}

void IMSCdata::clearData(){
    impedances.clear();
    voltages.clear();
    other.clear();
    eit.clear();
    eit.reserve(EIT_DATA_VECTOR_LENGTH);
}


// IMSC

ImpedanceMultiSensingCircuit::ImpedanceMultiSensingCircuit(QObject *parent) :
    QObject(parent),
    dataStorage(),
    offsetAngleComplexImpedance(0),
    offsetAngleComplexVoltage(0),
    gainU(std::numeric_limits<double>::quiet_NaN()),
    gainZ(std::numeric_limits<double>::quiet_NaN()),
    compensation_V_Re(std::numeric_limits<double>::quiet_NaN()),
    compensation_V_Im(std::numeric_limits<double>::quiet_NaN()),
    compensation_Z_Re(std::numeric_limits<double>::quiet_NaN()),
    compensation_Z_Im(std::numeric_limits<double>::quiet_NaN()),
    shunt_Impedance_Re(std::numeric_limits<double>::quiet_NaN()),
    shunt_Impedance_Im(std::numeric_limits<double>::quiet_NaN()),
    finishedDataAcquisition(false)
{
    //qDebug() << "ImpedanceMultiSensingCircuit-Thread-ID @ construct: " << QThread::currentThreadId();
    devConnected = false;
    serialPort = new QSerialPort();
    connect(serialPort, &QSerialPort::errorOccurred, this, &ImpedanceMultiSensingCircuit::handleSerialPortError);
}


ImpedanceMultiSensingCircuit::~ImpedanceMultiSensingCircuit() {}



/*!
    Opens Serial communication for command- and data-exchange between software and hardware.

    @param portName : COM-port of hardware

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::begin(const QString& portName) {
    //qDebug() << "ImpedanceMultiSensingCircuit-Thread-ID @ begin: " << QThread::currentThreadId();
    comPortDevName = portName;
    // Enumerate available serial ports
    QList<QSerialPortInfo> ports = QSerialPortInfo::availablePorts();
    // Find the port with 'portName' in the description
    QString selectedPort = "None";
    for (const QSerialPortInfo& port : ports) {
        if (port.description().contains(comPortDevName, Qt::CaseInsensitive)) {
            selectedPort = port.portName();
            break;
        }
    }
    // Check if port was found:
    if(QString::compare(selectedPort, "None")==0){
        return false;
    }
    // Serial port settings:
    serialPort->setPortName(selectedPort);
    serialPort->setBaudRate(QSerialPort::Baud115200); // Adjust baud rate as needed
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);
    serialPort->setFlowControl(QSerialPort::NoFlowControl);
        //serialPort->setFlowControl(QSerialPort::HardwareControl); // Not necessary for ATMEGA 32U4
    // Open port:
    if (!serialPort->open(QIODevice::ReadWrite)) {
        qDebug() << "Error opening serial port: " << serialPort->errorString();
        return false;
    }
    // Necessary for serial communication with ATMEGA 32U4:
    serialPort->setDataTerminalReady(true);
        //serialPort->setRequestToSend(true);
    // Double-check:
    if (!(serialPort->isOpen() && serialPort->isWritable() && serialPort->isReadable())){
        serialPort->close();
        return false;
    }
    // Recalibrate:
    if(!calibration()){
        serialPort->close();
        return false;
    }
    // Return success:
    devConnected = true;
    return true;
}

/*!
     Handle serial port errors / disconnections.
*/
void ImpedanceMultiSensingCircuit::handleSerialPortError(QSerialPort::SerialPortError error){
    if(error == QSerialPort::NoError) return; // No error
    qDebug() << "Serial port error: " << error;
    if(error == QSerialPort::TimeoutError || error == QSerialPort::ResourceError){
        serialPort->close();
        devConnected = false;
        // Any further actions required?
    }
}

/*!
    Starts the Signal Generator of the AD5933.
    It is needed to measure via 'measureVoltage' and 'measureImpedance'.
    More complex measurement routines manage this on their own.

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::manual_Start_MeasurementOscillation() {
    if(serialPort->write(QByteArray("B"))<1) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;
    return true;
}

/*!
     Stops the Signal Generator of the AD5933.
     More complex measurement routines manage this on their own.

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::manual_Stop_MeasurementOscillation() {
    if(serialPort->write(QByteArray("X"))<1) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;
    return true;
}

/*!
    Transmit the Stimulation Pattern from 'self.stimPattern' to the Impedance-Multi-Sensing-Circuit (needed for Electrical Impedance Tomography).

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::sendStimPatternToHardware(const int stimPattern[ELECTRODE_NUMBER][ELECTRODE_NUMBER]) {
    QByteArray byteData(""); byteData.append('\0');
    for (int j = 0; j < ELECTRODE_NUMBER; ++j) {
        if(serialPort->write(QByteArray("S"))<1) return false;
        byteData[0] = j;
        if(serialPort->write(byteData)<1) return false;
        for (int k = 0; k < ELECTRODE_NUMBER; ++k) {
            byteData[0] = stimPattern[j][k];
            if(serialPort->write(byteData)<1) return false;
        }
    }
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS*10)) return false;
    return true;
}

/*!
    Transmits the Measurement Pattern from measPattern to the Impedance-Multi-Sensing-Circuit (needed for Electrical Impedance Tomography).

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::sendMeasPatternToHardware(const int measPattern[ELECTRODE_NUMBER][MEASURE_VOLTAGES_PER_STIMULATION][ELECTRODE_NUMBER]) {
    QByteArray byteData(""); byteData.append('\0');
    for (int j = 0; j < ELECTRODE_NUMBER; ++j) {
        if(serialPort->write(QByteArray("M"))<1) return false;
        byteData[0] = j;
        if(serialPort->write(byteData)<1) return false;
        for (int k = 0; k < MEASURE_VOLTAGES_PER_STIMULATION; ++k) {
            for (int l = 0; l < ELECTRODE_NUMBER; ++l) {
                byteData[0] = measPattern[j][k][l];
                if(serialPort->write(byteData)<1) return false;
            }
        }
    }
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS*100)) return false;
    return true;
}

/*!
    Sets the measurement frequency of the hardware. Only Values between 2 and 99 are accepted.

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::setFrequency(uint8_t frequency, bool recalibrate) {
    frequency = std::clamp((int)frequency, 2, 99);
    if(serialPort->write(QByteArray("F"))<1) return false;
    QByteArray frequencyByteData; frequencyByteData.append(frequency);
    if(serialPort->write(frequencyByteData)<1) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;
    if(recalibrate){
        if(!calibration()) return false;
    }
    return true;
}

/*!
    Enable/Disable the voltage leakage correction of the hardware.

    @param onOff : 'true' activates the leakage correction, 'false' deactivates it.

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::voltageLeakageCorrection(bool onOff) {
    if(serialPort->write(onOff ? QByteArray("L") : QByteArray("l"))<1) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;
    return true;
}

/*!
    Set Pinmode of whole GPIO-port. 0 for input, 1 for output.

    @param port : Port of GPIO to set (1,2 or 3)
    @param value : Byte to set Pinmode (output--> 1, input-->0). e.g. gpioMode(2, 0b00000011]) sets the first two pins of GPIO 2 as output and the remaining as input. Missing values are padded with 0 (input), unnecessary values are ignored.

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::gpioMode(int port, std::byte value) {
    // Validate port number (e.g., 1, 2, or 3)
    if (port < 1 || port > 3) {
        // Handle invalid port error (e.g., throw exception or log error)
        return false;
    }
    // Send command to set pin mode
    if(serialPort->write(QByteArray("%"))<1) return false;
    if(serialPort->write(QByteArray("d"))<1) return false;
    QByteArray byteData; byteData.append(port);
    if(serialPort->write(byteData)<1) return false;
    byteData[0] = (uint8_t)value;
    if(serialPort->write(byteData)<1) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS))return false;
    return true;
}

/*!
    Reads Pin-values of whole GPIO-port. 0 for LOW, 1 for HIGH.

    @param port : Port of GPIO to read (1,2 or 3)

    @return values : flags of Pin values. e.g. gpioRead(2) read the Pin values of GPIO 2. [0/1, 0/1, 0/1, 0/1, 0/1, 0/1, 0/1, 0/1]. Only lower segnificant byte contains data.
                     in case of an error, negatice max is returned (INT16_MIN = -32768).
*/
int16_t ImpedanceMultiSensingCircuit::gpioRead(uint8_t port) {
    // Validate port number (e.g., 1, 2, or 3)
    if (port < 1 || port > 3) {
        // Handle invalid port error (e.g., throw exception or log error)
        return INT16_MIN;
    }
    // Send command to read pin values
    if(serialPort->write(QByteArray("%"))<1) return INT16_MIN;
    if(serialPort->write(QByteArray("r"))<1) return INT16_MIN;
    QByteArray portByteData; portByteData.append(port);
    if(serialPort->write(portByteData)<1) return INT16_MIN;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return INT16_MIN;

    // Read response with pin values (assuming response is a single byte)
    QByteArray data = serialPort->read(1);
    if (data.isEmpty()) {
        // Handle read error (e.g., throw exception or log error)
        return INT16_MIN;
    }
    // Return data
    return (int16_t) data[0];
}

/*!
    Writes to Pins of whole GPIO-port. 0 for LOW, 1 for HIGH.

    @param port : Port of GPIO to write (1,2 or 3)

    @param value : byte of Pin values to write. e.g. gpioWrite(2, 0b00000011) sets the first two pins of GPIO 2 to HIGH and the remaining to LOW. Unnecessary values are ignored

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::gpioWrite(int port, std::byte value) {
    // Validate port number (e.g., 1, 2, or 3)
    if (port < 1 || port > 3) {
        // Handle invalid port error (e.g., throw exception or log error)
        return false;
    }
    // Send command to write pin values
    if(serialPort->write(QByteArray("%"))<1) return false;
    if(serialPort->write(QByteArray("w"))<1) return false;
    QByteArray byteData; byteData.append(port);
    if(serialPort->write(byteData)<1) return false;
    byteData[0] = (uint8_t)value;
    if(serialPort->write(byteData)<1) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;
    return true;
}

/*!
    Reads calibration data from device and measures voltage offset angle to compensate the device's shift in phase.
*/
bool ImpedanceMultiSensingCircuit::calibration() {
    // Clear old data
    dataStorage.other.clear();
    // Send command to start measuring calibration data
    //qDebug() << "start Calib";
    if(serialPort->write(QByteArray("K"))<1) return false;
    if(!serialPort->flush()) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;
    // Store received data until finish flag arrives
    if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)) return false;
    finishedDataAcquisition = false;
    while (!finishedDataAcquisition) {
        int av =  serialPort->bytesAvailable();
        if (av) {
            if(!readDataPoint()) return false;
        }else{
            if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)) return false;
        }
    }
    // Extract calibration data from received data (assuming data structure)
    gainU = dataStorage.other[0].magnitude;  // Check if keys exist
    gainZ = dataStorage.other[1].magnitude;
    compensation_V_Re = dataStorage.other[2].realValue;
    compensation_V_Im = dataStorage.other[2].imagValue;
    compensation_Z_Re = dataStorage.other[3].realValue;
    compensation_Z_Im = dataStorage.other[3].imagValue;
    shunt_Impedance_Re = dataStorage.other[4].realValue;
    shunt_Impedance_Im = dataStorage.other[4].imagValue;
    // Calculate shunt impedance
    std::complex<double>Z_shunt = std::complex<float>(shunt_Impedance_Re, shunt_Impedance_Im);
    // Calculate rotation offset (assuming data format)
    offsetAngleComplexImpedance = std::arg(Z_shunt);
    // Start measurement oscillation (assuming implemented)
    if(!manual_Start_MeasurementOscillation()) return false;
    QThread::msleep(10); // Use std::this_thread for sleep
    // Measure voltage for angle offset (assuming implemented)
    if(std::isnan(measureVoltage(0, -1, 0, complex))) return false;
    std::complex<float> u0(dataStorage.voltages.back().realValue, dataStorage.voltages.back().imagValue);
    // Calculate voltage offset angle (assuming data format)
    offsetAngleComplexVoltage = std::arg(u0);
    // Stop measurement oscillation (assuming implemented)
    if(!manual_Stop_MeasurementOscillation()) return false;
    // Return success:
    //qDebug() << "end Calib";
    return true;
}

/*!
    Measures all meaninfull combinations of voltages and impedances(2601 Voltages, 136 impedances) across all 16 electrodes plus one needle.
    Measurements get stored in the dataStorage of the class.

    @param measMod : measurement mode (magnitude / complex)

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::measureAll(measureMode measMod) {
    // Clear old data
    dataStorage.voltages.clear();
    dataStorage.impedances.clear();

    // Send command based on measure mode
    if (measMod == complex) {
        if(serialPort->write(QByteArray("*"))<1) return false;
    } else if (measMod == magnitude) {
        if(serialPort->write(QByteArray("+"))<1) return false;
    } else {
        // Handle invalid measure mode (e.g., throw exception or log error)
        return false;
    }
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;

    // Store received data until finish flag arrives

    if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)) return false;
    finishedDataAcquisition = false;
    while (!finishedDataAcquisition) {
        int av =  serialPort->bytesAvailable();
        if (av) {
            if(!readDataPoint()) return false;
        }else{
            if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)) return false;
        }
    }
    return true;
}

/*!
    Collects all necessary measurements to calculate Electrical Impedance Tomography (EIT).

    @param measMod : measurement mode (magnitude / complex)

    @return : vector of voltage differences for EIT calculation, in case of an error a vector of NAN is returned
*/
std::vector<float> ImpedanceMultiSensingCircuit::measureEIT(eitMode eitMod) {
    // Clear old data
    dataStorage.eit.clear();

    // Initialize empty vector for results
    std::vector<float> v;
    v.reserve(EIT_DATA_VECTOR_LENGTH);

    // Send command based on measure mode
    if (eitMod == raw) {
        if(serialPort->write(QByteArray("c"))<1){
            std::vector<float> nan_vector(EIT_DATA_VECTOR_LENGTH, std::numeric_limits<float>::quiet_NaN());
            return nan_vector;
        }
    } else if (eitMod == corrected) {
        if(serialPort->write(QByteArray("C"))<1){
            std::vector<float> nan_vector(EIT_DATA_VECTOR_LENGTH, std::numeric_limits<float>::quiet_NaN());
            return nan_vector;
        }
    } else {
        std::vector<float> nan_vector(EIT_DATA_VECTOR_LENGTH, std::numeric_limits<float>::quiet_NaN());
        return nan_vector;
    }
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)){
        std::vector<float> nan_vector(EIT_DATA_VECTOR_LENGTH, std::numeric_limits<float>::quiet_NaN());
        return nan_vector;
    }

    // Store received data until finish flag arrives
    if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)){
        std::vector<float> nan_vector(EIT_DATA_VECTOR_LENGTH, std::numeric_limits<float>::quiet_NaN());
        return nan_vector;
    }
    finishedDataAcquisition = false;
    while (!finishedDataAcquisition) {
        int av =  serialPort->bytesAvailable();
        if (av) {
            if(!readDataPoint()){
                std::vector<float> nan_vector(EIT_DATA_VECTOR_LENGTH, std::numeric_limits<float>::quiet_NaN());
                return nan_vector;
            }
            if (!finishedDataAcquisition) {
                v.push_back(dataStorage.eit.back().magnitude);  // Access last element of "eit" storage
            }
        }else{
            if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)){
                std::vector<float> nan_vector(EIT_DATA_VECTOR_LENGTH, std::numeric_limits<float>::quiet_NaN());
                return nan_vector;
            }
        }
    }

    return v; // Return vector of measured voltages
}

/*!
    Get a voltage measurement at electrode "measEl", signal source is electrode "fromEl", signal sink is electrode "toEl".
    If "complex" gets requested, complex values are only available in the stored data ("dataStorage.impedances.back()").

    @param fromEl : Source electrode providing the signal
    @param toEl : Sink electrode providing ground
    @param measEl : Electrode for measuring voltage
    @param measMod : measurement mode (magnitude / complex)

    @return : voltage to be measured, NAN in case an error occured
*/
float ImpedanceMultiSensingCircuit::measureVoltage(int fromEl, int toEl, int measEl, measureMode measMod) {
    // Determine data type based on measure mode
    pollType pType = (measMod == magnitude) ? voltageMagnitude  : voltageComplex ;
    // Poll data using internal function (assuming implemented)
    if(!pollMeasurement(pType, fromEl, toEl, measEl)){
        emit serialPortError();
        return NAN;
    }
    // Read data point
    if(!readDataPoint()){
        emit serialPortError();
        return NAN;
    }
    // If something went wrong:
    if (dataStorage.voltages.size() < 1) {
        // Handle potential error (e.g., no data received)
        return NAN; // Or throw exception
    }
    // Return magnitude anyway
    return (measMod == magnitude) ? dataStorage.voltages.back().magnitude : (qSqrt(qPow(dataStorage.voltages.back().realValue, 2) + qPow(dataStorage.voltages.back().realValue, 2)));
}

/*!
    Get an impedance measurement between electrode "fromEl" and electrode "toEl".
    If "complex" gets requested, complex values are only available in the stored data ("dataStorage.impedances.back()").

    @param fromEl : Source electrode providing the signal
    @param toEl : Sink electrode providing ground
    @param measMod : measurement mode (magnitude / complex)

    @return : impedance to be measured, NAN in case an error occured
*/
float ImpedanceMultiSensingCircuit::measureImpedance(int fromEl, int toEl, measureMode measMod) {
    // Determine data type based on measure mode
    pollType pType = (measMod == magnitude) ? impedanceMagnitude : impedanceComplex;

    // Poll data using internal function (assuming implemented)
    if(!pollMeasurement(pType, fromEl, toEl)){
        emit serialPortError();
        return NAN;
    }
    // Read data point
    if(!readDataPoint()){
        emit serialPortError();
        return NAN;
    }

    // Extract data based on data structure (assuming "impedances" map)
    if (dataStorage.impedances.size() < 1) {
        // Handle potential error (e.g., no data received)
        return NAN; // Or throw exception
    }
    // Return magnitude anyway
    return (measMod == magnitude) ? dataStorage.impedances.back().magnitude : (qSqrt(qPow(dataStorage.impedances.back().realValue, 2) + qPow(dataStorage.impedances.back().realValue, 2)));
}

/*!
    Requests a measurement from the hardware. Should only be used internally.

    @param mode : Internal code to tell the IMSC-hardware which type of measurement needs to be taken
    @param fromEl : Source electrode providing the signal
    @param toEl : Sink electrode providing ground
    @param measEl : Electrode for measuring voltage

    @return : true if successfull, false if error occured
*/
/*void ImpedanceMultiSensingCircuit::pollMeasurement(pollType mode, int fromEl, int toEl, int measEl) {
    // Send measurement request
    serialPort->write(QByteArray("@"));
    QByteArray byteData(""); byteData.append(mode);
    serialPort->write(byteData);

    byteData[0] = fromEl;
    serialPort->write(byteData);

    byteData[0] = toEl;
    serialPort->write(byteData);

    byteData[0] = measEl;
    serialPort->write(byteData);
    serialPort->flush();
    serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS);
}*/
bool ImpedanceMultiSensingCircuit::pollMeasurement(pollType mode, int fromEl, int toEl, int measEl) {
    // Send measurement request
    if(serialPort->write(QByteArray("@"))<1) return false;

    QByteArray byteData(""); byteData.append(mode);
    if(serialPort->write(byteData)<1) return false;

    byteData[0] = fromEl;
    if(serialPort->write(byteData)<1) return false;

    byteData[0] = toEl;
    if(serialPort->write(byteData)<1) return false;

    byteData[0] = measEl;
    if(serialPort->write(byteData)<1) return false;

    if(!serialPort->flush()) return false;
    if(!serialPort->waitForBytesWritten(SERIAL_WAIT_FOR_WRITE_TIMEOUT_MS)) return false;

    return true;
}

/*!
    Converts four Bytes of Data into a float-number (IEEE 754)

    @param c : Vector of four Byte.

    @return : Data converted to float.
*/
double ImpedanceMultiSensingCircuit::byte4float(const std::vector<uint8_t>& c) {
    // Extract exponent and mantissa
    int E = ((c[3] & 127) << 1) + ((c[2] & 128) >> 7);
    int m = c[0] + (c[1] << 8) + ((c[2] & 127) << 16);

    // Calculate float value
    double v = (1 + pow(2.0, -23) * m) * pow(2.0, E - 127);

    // Handle sign bit
    if (c[3] > 127) {
        return -v;
    } else {
        return v;
    }
}

/*!
    Read a single data point from serial port.

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::readDataPoint() {
    //qDebug() << "readDataPoint()";
  // Read mode byte from serial port

    if(!serialPort->bytesAvailable()){
        if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)) return false;
    }


    QByteArray sDataBytes = serialPort->read(1);
    if (sDataBytes.isEmpty()) {
        //throw HardwareError("Serial communication: no data error.");
        return false;
    }
    pollType mode;
    switch (static_cast<int>(sDataBytes[0])) {
        case 0:  mode = endP; break;
        case 1:  mode = voltageMagnitude; break;
        case 2:  mode = voltageComplex; break;
        case 3:  mode = impedanceMagnitude; break;
        case 4:  mode = impedanceComplex; break;
        case 5:  mode = otherMagnitude; break;
        case 6:  mode = otherComplex; break;
        case 7:  mode = eitMagnitude; break;
        default: mode = communicationError; break;
    }

    // Check for end of transmission
    if (mode == endP) {
        finishedDataAcquisition = true;
        return true;
        }

    // Validate mode range
    if (mode == communicationError) {
        //throw HardwareError("Invalid data type received.");
        return false;
    }

    // Determine data size based on complexity
    int dataSize = (mode % 2 == 0) ? 11 : 7;

    // Wait if necessary:
    if(serialPort->bytesAvailable()<dataSize){
        if(!serialPort->waitForReadyRead(SERIAL_WAIT_FOR_READ_TIMEOUT_MS)) return false;
    }
    // Read data bytes
    sDataBytes = serialPort->read(dataSize);
    if (sDataBytes.isEmpty()) {
        //throw HardwareError("Serial communication error.");
        return false;
    }

    // Extract data based on mode
    int fromEl = (static_cast<uchar>(sDataBytes[0]) >= 0 && static_cast<uchar>(sDataBytes[0]) <= ELECTRODE_NUMBER + NEEDLE_NUMBER) ? sDataBytes[0] : -1;
    int toEl = (static_cast<uchar>(sDataBytes[1]) >= 0 && static_cast<uchar>(sDataBytes[1]) <= ELECTRODE_NUMBER + NEEDLE_NUMBER) ? sDataBytes[1] : -1;
    int measureEl = (mode == eitMagnitude || (static_cast<uchar>(sDataBytes[2]) >= 0 && static_cast<uchar>(sDataBytes[2]) <= ELECTRODE_NUMBER + NEEDLE_NUMBER)) ? sDataBytes[2] : -1;


    if (mode % 2 == 0) { // Complex data
        float realValue1 = byte4float(std::vector<uint8_t>(sDataBytes.begin() + 3, sDataBytes.begin() + 7));
        float imagValue1 = byte4float(std::vector<uint8_t>(sDataBytes.begin() + 7, sDataBytes.end()));
        std::complex<float> complexValue;

        // Handle complex voltage/impedance correction
        if (mode == voltageComplex) { // Complex voltage
          complexValue = correctComplexByRotation(offsetAngleComplexVoltage, realValue1, -imagValue1);
        } else if (mode == impedanceComplex) { // Complex impedance
          complexValue = correctComplexByRotation(offsetAngleComplexImpedance, realValue1, imagValue1);
        }

        // Append data to appropriate list based on mode
        switch (mode) {
            case voltageComplex: // Complex voltage
                dataStorage.voltages.push(imscDataset(fromEl, toEl, measureEl, realValue1, imagValue1, std::numeric_limits<float>::quiet_NaN()));
                break;
            case impedanceComplex: // Complex impedance
                dataStorage.impedances.push(imscDataset(fromEl, toEl, -1, realValue1, imagValue1, std::numeric_limits<float>::quiet_NaN()));
                break;
            case otherComplex: // Complex other data
                dataStorage.other.push(imscDataset(fromEl, toEl, measureEl, realValue1, imagValue1, std::numeric_limits<float>::quiet_NaN()));
                break;
            default:
                break;
        }
    } else { // Magnitude Data
        float magnitudeValue1 = byte4float(std::vector<uint8_t>(sDataBytes.begin() + 3, sDataBytes.end()));

        // Implement your logic for handling different modes based on your changes
        switch (mode) {
            case voltageMagnitude: // Magnitude of Voltage:
                dataStorage.voltages.push(imscDataset(fromEl, toEl, measureEl, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), magnitudeValue1));
                break;
            case impedanceMagnitude: // Magnitude of Impedance:
                dataStorage.impedances.push(imscDataset(fromEl, toEl, -1, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), magnitudeValue1));
                break;
            case otherMagnitude: // Magnitude of other Data:
                dataStorage.other.push(imscDataset(fromEl, toEl, measureEl, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), magnitudeValue1));
                break;
            case eitMagnitude: // EIT Data (Magnitude):
                dataStorage.eit.push_back(imscDataset(fromEl, toEl, measureEl, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::quiet_NaN(), magnitudeValue1));
                break;
            default:
                // Handle unexpected mode (optional)
                break;
        }
    }
    // Return true on success:
    return true;
}

/*!
    Correct complex results for the unwanted phase shift of the hardware itself.

    @param angle : hardware-offset-angle to be compensated

    @param real : real part of measurement

    @param imag : imaginary part of measurement

    @return : corrected, complex result
*/
std::complex<float> ImpedanceMultiSensingCircuit::correctComplexByRotation(float angle, float real, float imag) {
    std::complex<float> complexNumber(real, imag);
    double absV = std::abs(complexNumber);
    double angleV = std::arg(complexNumber) - angle;
    return std::complex<float>(absV * std::cos(angleV), absV * std::sin(angleV));
}

/*!
    Check connection to device.

    @return : true if device is connected, false if disconnected
*/
bool ImpedanceMultiSensingCircuit::checkConnection() {
    return devConnected;
}

/*!
    End communication with hardware.

    @return : true if successfull, false if error occured
*/
bool ImpedanceMultiSensingCircuit::end() {
    manual_Stop_MeasurementOscillation();
    serialPort->close();
    return true;
}
