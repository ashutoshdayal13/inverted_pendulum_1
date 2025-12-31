import os
import sys
dir_path = os.path.dirname(os.path.realpath(__file__))
sys.path.append(os.path.abspath(os.path.join(dir_path, "./lib")))

# import importlib
# py_major_version = sys.version_info[0]
# py_minor_version = sys.version_info[1]
# globals()["nmotion_transport"] = importlib.import_module(f'libnmotion_transport_python_{py_major_version}{py_minor_version}')
# globals()["nmotion_transport"] = importlib.import_module(f'libnmotion_transport_python')

import libnmotion_transport_python as nmotion_transport
from .UsbInterface import USBInterface
from typing import Any

class IMU():
    """! IMU class which wraps around the IMU C++ class
    
    This class provides functionality to interact with IMU devices through the CAN interface.
    """
    __imu: Any = None
    id: int = None
    
    def __init__(self, id: int = -1, interface: USBInterface = None):

        """!
        IMU class constructor
        
        @param id CAN Node ID of the IMU device
        @param interface Initialised CANInterface or USBInterface object
        """
        if(not interface):
            raise Exception("An USBInterface object needs to be passed")

        if(id < 0):
            try:
                self.id = 0
                interface = interface
                self.__imu = nmotion_transport.IMU(interface.getInterface())
            except TypeError:
                raise Exception("Windows does not suppport using NMotion Components via native USB port. Use it's CAN Interface instead with NLink Adapter.")
        else:
            self.id = id
            interface = interface
            self.__imu = nmotion_transport.IMU(id, interface.getInterface())
    
    def getHardwareVersion(self) -> 'tuple[int, int, int, int, int]':
        """!
        Get the IMU hardware's current tag, major, minor and variant information.

        @return Tuple containing return status, tag, major, minor and variant information.
        """
        return self.__imu.getHardwareVersion()

    def getFirmwareVersion(self) -> 'tuple[int, int, int, int]':
        """!
        Get the IMU firmware's major, minor and revision information.

        @return Tuple containing return status, major, minor and revision information.
        """
        return self.__imu.getFirmwareVersion()

    def getFirmwareCommit(self) -> 'tuple[int, str]':

        """!
        Get the IMU firmware's commit hash.
        
        @return Tuple containing return status and the commit hash string.
        """
        return self.__imu.getFirmwareCommit()
    
    def getRotationVectorData(self) -> 'tuple[float, float, float, float]':
        """!
        Retrieve the rotation vector data from the IMU
        
        @return Tuple containing (quat_i, quat_j, quat_k, quat_real) quaternion components
        """
        return self.__imu.getRotationVectorData()

    def getRotationVectorAccuracy(self) -> 'tuple[float, int]':
        """!
        Retrieve the rotation vector accuracy data from the IMU
        
        @return Tuple containing (quat_rad_accuracy, quat_accuracy) rotation vector accuracy
        """
        return self.__imu.getRotationVectorAccuracy()

    def getRawAccelerometerData(self) -> 'tuple[int, int, int]':
        """!
        Retrieve the raw accelerometer data from the IMU
        
        @return Tuple containing raw accelerometer values (x, y, z)
        """
        return self.__imu.getRawAccelerometerData()
    
    def getRawGyroData(self) -> 'tuple[int, int, int]':
        """!
        Retrieve the raw gyroscope data from the IMU
        
        @return Tuple containing raw gyro values (x, y, z)
        """
        return self.__imu.getRawGyroData()
    
    def getRawMagnetometerData(self) -> 'tuple[int, int, int]':
        """!
        Retrieve the raw magnetometer data from the IMU
        
        @return Tuple containing raw magnetometer values (x, y, z)
        """
        return self.__imu.getRawMagnetometerData()

    def getLinearAccelerometerData(self) -> 'tuple[float, float, float, int]':
        """!
        Retrieve the linear accelerometer data from the IMU
        
        @return Tuple containing linear acceleration values (x, y, z) in m/s² and accuracy
        """
        return self.__imu.getLinearAccelerometerData()

    def getAccelerometerData(self) -> 'tuple[float, float, float, int]':
        """!
        Retrieve the accelerometer data from the IMU
        
        @return Tuple containing acceleration values (x, y, z) in m/s² and accuracy
        """
        return self.__imu.getAccelerometerData()
    
    def getGyroData(self) -> 'tuple[float, float, float, int]':
        """!
        Retrieve the gyroscope data from the IMU
        
        @return Tuple containing gyro values (x, y, z) in radians per second and accuracy
        """
        return self.__imu.getGyroData()

    def getMagnetometerData(self) -> 'tuple[float, float, float, int]':
        """!
        Retrieve the magnetometer data from the IMU
        
        @return Tuple containing magnetometer values (x, y, z) in uTesla and accuracy
        """
        return self.__imu.getMagnetometerData()

    def getUncalibratedGyroData(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the uncalibrated gyroscope data from the IMU
        
        @return Tuple containing uncalibrated gyro values (x, y, z) in radians per second
        """
        return self.__imu.getUncalibratedGyroData()

    def getUncalibratedGyroBiasData(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the uncalibrated gyroscope bias data from the IMU
        getEncoderRawData
        @return Tuple containing uncalibrated gyro bias values (x, y, z) in radians per second
        """
        return self.__imu.getUncalibratedGyroBiasData()

    def getEulerAngles(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the Euler angles from the IMU
        
        @return Tuple containing Euler angles (yaw, pitch, roll) in degrees
        """
        return self.__imu.getEulerAngles()

    def getGravityData(self) -> 'tuple[float, float, float]':
        """!
        Retrieve the gravity data from the IMU
        
        @return Tuple containing gravity values (x, y, z) in g
        """
        return self.__imu.getGravityData()

    def enableRotationVector(self, time_interval: int) -> int:
        """!
        Enable rotation vector data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRotationVector(time_interval)
    
    def disableRotationVector(self) -> int:
        """!
        Disable rotation vector data
        
        @return Return status
        """
        return self.__imu.disableRotationVector()
    
    def enableRawAccelerometer(self, time_interval: int) -> int:
        """!
        Enable raw accelerometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRawAccelerometer(time_interval)
    
    def disableRawAccelerometer(self) -> int:
        """!
        Disable raw accelerometer data

        @return Return status
        """
        return self.__imu.disableRawAccelerometer()

    def enableRawGyro(self, time_interval: int) -> int:
        """!
        Enable raw gyroscope data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRawGyro(time_interval)
    
    def disableRawGyro(self) -> int:
        """!
        Disable raw gyroscope data
        
        @return Return status
        """
        return self.__imu.disableRawGyro()

    def enableRawMagnetometer(self, time_interval: int) -> int:
        """!
        Enable raw magnetometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableRawMagnetometer(time_interval)
    
    def disableRawMagnetometer(self) -> int:
        """!
        Disable raw magnetometer data
        
        @return Return status
        """
        return self.__imu.disableRawMagnetometer()

    def enableLinearAccelerometer(self, time_interval: int) -> int:
        """!
        Enable linear accelerometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableLinearAccelerometer(time_interval)
    
    def disableLinearAccelerometer(self) -> int:
        """!
        Disable linear accelerometer data
        
        @return Return status
        """
        return self.__imu.disableLinearAccelerometer()

    def enableAccelerometer(self, time_interval: int) -> int:
        """!
        Enable accelerometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableAccelerometer(time_interval)
    
    def disableAccelerometer(self) -> int:
        """!
        Disable accelerometer data
        
        @return Return status
        """
        return self.__imu.disableAccelerometer()

    def enableGyro(self, time_interval: int) -> int:
        """!
        Enable gyroscope data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableGyro(time_interval)
    
    def disableGyro(self) -> int:
        """!
        Disable gyroscope data
        
        @return Return status
        """
        return self.__imu.disableGyro()

    def enableMagnetometer(self, time_interval: int) -> int:
        """!
        Enable magnetometer data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableMagnetometer(time_interval)
    
    def disableMagnetometer(self) -> int:
        """!
        Disable magnetometer data
        
        @return Return status
        """
        return self.__imu.disableMagnetometer()

    def enableUncalibratedGyro(self, time_interval: int) -> int:
        """!
        Enable uncalibrated gyroscope data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableUncalibratedGyro(time_interval)
    
    def disableUncalibratedGyro(self) -> int:
        """!
        Disable uncalibrated gyroscope data
        
        @return Return status
        """
        return self.__imu.disableUncalibratedGyro()
    
    def enableGravity(self, time_interval: int) -> int:
        """!
        Enable gravity data and configure it
        
        @param  Report interval in milliseconds
        @return Return status
        """
        return self.__imu.enableGravity(time_interval)
    
    def disableGravity(self) -> int:
        """!
        Disable gravity data
        
        @return Return status
        """
        return self.__imu.disableGravity()
        
    def setNodeId(self, id:int) -> int:
        """!
        Set the node id for the NIMU

        @param id Device ID of the NIMU

        @return Return Status
        """
        return self.__imu.setNodeId(id)

    def getNodeId(self) -> 'tuple[int, int]':
        """!
        Get current Node ID of the NIMU

        @return Tuple containing return status and the current Node ID.
        """
        return self.__imu.getNodeId()
    

    def saveConfigurations(self) -> int:
        """!
        Save the current configurations to non-volatile memory
        
        @return Return status
        """
        return self.__imu.saveConfigurations()

    def eraseConfigurations(self) -> int:
        """!
        Erase the saved configurations from non-volatile memory
        
        @return Return status
        """
        return self.__imu.eraseConfigurations()

    def startCalibration(self, accel: bool, gyro: bool, mag: bool, planarAccel: bool) -> int:
        """!
        Start calibration process and configure it
        
        @param accel Enable accelerometer calibration
        @param gyro Enable gyroscope calibration
        @param mag Enable magnetometer calibration
        @param planarAccel Enable planar accelerometer calibration
        @return Return status
        """
        return self.__imu.startCalibration(accel, gyro, mag, planarAccel)

    def stopCalibration(self) -> int:
        """!
        Stop the ongoing calibration process
        
        @return Return status
        """
        return self.__imu.stopCalibration()

    def saveCalibration(self) -> int:
        """!
        Save calibration data to non-volatile memory
        
        @return Return status
        """
        return self.__imu.saveCalibration()

    def getCalibrationStatus(self) -> 'tuple[int, bool]':
        """!
        Get the current calibration status
        
        @return Tuple containing return status and calibration completion status
        """
        return self.__imu.getCalibrationStatus()
    
    def clearDCD(self) -> int:
        """!
        Clear Dynamic Calibration Data (DCD)
        
        @return Return status
        """
        return self.__imu.clearDCD()

    def tareYaw(self) -> int:
        """!
        Apply tare to the Z-axis (yaw)
        
        @return Return status
        """
        return self.__imu.tareYaw()

    def tareAll(self) -> int:
        """!
        Apply tare to all axes of the IMU
        
        @return Return status
        """
        return self.__imu.tareAll()
    
    def clearTare(self) -> int:
        """!
        Clear all tare offsets
        
        @return Return status
        """
        return self.__imu.clearTare()
    
    def setOrientation(self, i: float, j: float, k: float, real: float) -> int:
        """!
        Set the sensor orientation using a quaternion
        
        @param i The i component of the quaternion
        @param j The j component of the quaternion
        @param k The k component of the quaternion
        @param real The real component of the quaternion
        @return Return status
        """
        return self.__imu.setOrientation(i, j, k, real)
    
    def enterDFUMode(self) -> int:
        """!
        Enter Device Firmware Update Mode.

        @return Return Status.
        """
        return self.__imu.enterDFUMode()
    
    def rebootNIMU(self) -> int:
        """!
        Reboot the NIMU device.

        @return Return Status.
        """
        return self.__imu.rebootNIMU()
