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

class NCoder():
    """! NCoder class which wraps around the NCoder C++ class
    
    This class provides functionality to interact with NCoder devices through the CAN interface.
    """
    __ncoder: Any = None
    id: int = None
    
    def __init__(self, id: int = -1, interface: USBInterface = None):
        """!
        NCoder class constructor
        
        @param id CAN Node ID of the NCoder device
        @param interface Initialised CANInterface or USBInterface object
        """
        if(not interface):
            raise Exception("An USBInterface object needs to be passed")

        if(id < 0):
            try:
                self.id = 0
                interface = interface
                self.__ncoder = nmotion_transport.NCoder(interface.getInterface())
            except TypeError:
                raise Exception("Windows does not suppport using NMotion Components via native USB port. Use it's CAN Interface instead with NLink Adapter.")
        else:
            self.id = id
            interface = interface
            self.__ncoder = nmotion_transport.NCoder(id, interface.getInterface())
  
    def getHardwareVersion(self) -> 'tuple[int, int, int, int, int]':
        """!
        Get the NCoder hardware's current tag, major, minor and variant information.
        
        @return Tuple containing return status, tag, major, minor and variant information.
        """
        return self.__ncoder.getHardwareVersion()

    def getFirmwareVersion(self) -> 'tuple[int, int, int, int]':
        """!
        Get the NCoder firmware's major, minor and revision information.
        
        @return Tuple containing return status, major, minor and revision information.
        """
        return self.__ncoder.getFirmwareVersion()
    
    def getFirmwareCommit(self) -> 'tuple[int, str]':

        """!
        Get the NCoder firmware's commit hash.
        
        @return Tuple containing return status and the commit hash string.
        """
        return self.__ncoder.getFirmwareCommit()
    
    def setNodeId(self, id:int) -> int:
        """!
        Set the node id for the NCoder

        @param id Device ID of the NCoder

        @return Return Status
        """
        return self.__ncoder.setNodeId(id)

    def getNodeId(self) -> 'tuple[int, int]':
        """!
        Get current Node ID of the NCoder

        @return Tuple containing return status and the current Node ID.
        """
        return self.__ncoder.getNodeId()
    

    def configureEncoder(self,encoder_type: int, resolution_param: int = 4096) -> int:
        """!
        Configure the encoder parameters
        
        @param encoder_type Type of encoder used
        @param resolution_param Encoder resolution (in bit) 
        
        @return Return status
        """
        return self.__ncoder.configureEncoder(encoder_type, resolution_param)
    
    
    def getEncoderConfiguration(self) -> 'tuple[int, int, int]':
        """!
        Retrieve the current encoder configuration parameters
        
        @return Tuple containing return status, encoder type and resolution parameter (bits)
        """
        return self.__ncoder.getEncoderConfiguration()
    
          
    def setCurrentPositionToZero(self) -> int:
        """!
        Set the current position of the NCoder to zero

        @return Return status
        """
        return self.__ncoder.setCurrentPositionToZero()
    
    def getZeroPosition(self) -> 'tuple[int, float]':
        """!
        Get the zero position of the NCoder

        @return Tuple containing return status and the zero position value
        """
        return self.__ncoder.getZeroPosition()

    def setEncoderDirection(self, direction: int) -> int:
        """!
        Set the encoder direction for the NCoder
        
        @param direction Encoder direction to be set (0 for normal, 1 for inverted)
        
        @return Return status
        """
        return self.__ncoder.setEncoderDirection(direction)
    
    def getEncoderDirection(self) -> 'tuple[int, int]':
        """!
        Get the current encoder direction
        
        @return Tuple containing return status and the current encoder direction (0 for normal, 1 for inverted)
        """
        return self.__ncoder.getEncoderDirection()
    
    def setFilterWindowLength(self, filter_window_length: int) -> int:
        """!
        Set the filter window length for the NCoder

        @param filter_window_length Filter window length value to be set

        @return Return status
        """
        return self.__ncoder.setFilterWindowLength(filter_window_length)

    def getFilterWindowLength(self) -> 'tuple[int, int]':
        """!
        Get the filter window length of the NCoder
        
        @return Tuple containing return status and the filter window length value
        """
        return self.__ncoder.getFilterWindowLength()

    def setMAxxxSPIRegister(self, register_address: int, data: int) -> int:
        """!
        Set the maxxx encoder spi register
        
        @param register_address register_addres value to be which needs to be modified
        
        @return Tuple containing return status and the spi register data value value.
        """        
        return self.__ncoder.setMAxxxSPIRegister(register_address, data)

    def getMAxxxSPIRegister(self, register_address: int) -> 'tuple[int, int, bool]':
        """!
        Get the data from the maxxx encoder spi register

        @param register_address register address from which the data needs to be read
        @param 
        @param spi_read_status get the spi read status
                
        @return Tuple containing return status and the current encoder bct value value.
        """
        return self.__ncoder.getMAxxxSPIRegister(register_address)
    
    def setScalingFactor(self, scaling_factor: float) -> int:
        """!
        Set the scaling factor for the NCoder
        
        @param scaling_factor Scaling factor value to be set
        
        @return Return status
        """
        return self.__ncoder.setScalingFactor(scaling_factor)
    
    def getScalingFactor(self) -> 'tuple[int, float]':
        """!
        Get the scaling factor of the NCoder
        
        @return Tuple containing return status and the scaling factor value
        """
        return self.__ncoder.getScalingFactor()
    
    def getEncoderCount(self) -> 'tuple[int, int]':
        """!
        Get the current encoder count (angular Increment)
        
        @return Tuple containing return status and the encoder count value
        """
        return self.__ncoder.getEncoderCount()

    def getEncoderRawData(self) -> 'tuple[int, int]':
        """!
        Get the raw absolute position data directly from encoder hardware
        
        @return Tuple containing return status and the raw data value from the encoder
        """
        return self.__ncoder.getEncoderRawData()
    
    def getAbsoluteAngle(self) -> 'tuple[int, float]':
        """!
        Get the current absolute angle measurement from the encoder (in degrees)
        
        @return Tuple containing return status and the absolute angle value
        """
        return self.__ncoder.getAbsoluteAngle()
    
    def getMultiturnAngle(self) -> 'tuple[int, float]':
        """!
        Get the current multiturn angle of the NCoder
        
        @return Tuple containing return status and the multiturn angle value (in degrees)
        """
        return self.__ncoder.getMultiturnAngle()    
    
    def getRawVelocity(self) -> 'tuple[int, float]':
        """!
        Get the raw velocity of the NCoder
        
        @return Tuple containing return status and the raw velocity value (in degrees per second)
        """
        return self.__ncoder.getRawVelocity()
    
    def getScaledVelocity(self) -> 'tuple[int, float]':
        """!
        Get the scaled velocity of the NCoder
        
        @return Tuple containing return status and the scaled velocity value (in degrees per second)
        """
        return self.__ncoder.getScaledVelocity()
    
    def getFilteredVelocity(self) -> 'tuple[int, float]':
        """!
        Get the filtered velocity of the NCoder
        
        @return Tuple containing return status and the filtered velocity value (in degrees per second)
        """
        return self.__ncoder.getFilteredVelocity()
    
    def getScaledFilteredVelocity(self) -> 'tuple[int, float]':
        """!
        Get the scaled filtered velocity of the NCoder
        
        @return Tuple containing return status and the scaled filtered velocity value (in degrees per second)
        """
        return self.__ncoder.getScaledFilteredVelocity()
    
    def getRawAcceleration(self) -> 'tuple[int, float]':
        """!
        Get the acceleration of the NCoder
        
        @return Tuple containing return status and the acceleration value (in degrees per second squared)
        """
        return self.__ncoder.getRawAcceleration()

    def getScaledAcceleration(self) -> 'tuple[int, float]':
        """!
        Get the scaled acceleration of the NCoder
        
        @return Tuple containing return status and the scaled acceleration value (in degrees per second squared)
        """
        return self.__ncoder.getScaledAcceleration()
    
    def getFilteredAcceleration(self) -> 'tuple[int, float]':
        """!
        Get the filtered acceleration of the NCoder
        
        @return Tuple containing return status and the filtered acceleration value (in degrees per second squared)
        """
        return self.__ncoder.getFilteredAcceleration()
    
    def getScaledFilteredAcceleration(self) -> 'tuple[int, float]':
        """!
        Get the scaled filtered acceleration of the NCoder
        
        @return Tuple containing return status and the scaled filtered acceleration value (in degrees per second squared)
        """
        return self.__ncoder.getScaledFilteredAcceleration()
    
    def setLinearCount(self, count: int) -> int:
        """!
        Set the linear count for the NCoder
        
        @param count Linear count value to be set
        
        @return Return status
        """
        return self.__ncoder.setLinearCount(count)
    
    def getErrorCode(self) -> 'tuple[int, int]':
        """!
        Get any encoder errors
        
        @return Tuple containing return status and the error code value
        """
        return self.__ncoder.getErrorCode()

    def clearEncoderErrors(self) -> int:
        """!
        Clear any encoder errors
        
        @return Return status
        """
        return self.__ncoder.clearEncoderErrors()

    def rebootNCoder(self) -> int:
        """!
        Reboot the NCoder device
        
        @return Return status
        """
        return self.__ncoder.rebootNCoder()

    def saveConfigurations(self) -> int:
        """!
        Save the current configurations to non-volatile memory.

        @return Return Status .
        """
        return self.__ncoder.saveConfigurations()

    def eraseConfigurations(self) -> int:
        """!
        Erase the saved configurations from the non-volatile memory.

        @return Return Status.
        """
        return self.__ncoder.eraseConfigurations()
    
    def enterDFUMode(self) -> int:
        """!
        Enter Device Firmware Update Mode.

        @return Return Status.
        """
        return self.__ncoder.enterDFUMode()
