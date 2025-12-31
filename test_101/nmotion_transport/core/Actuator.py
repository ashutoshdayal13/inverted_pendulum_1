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

class Actuator():
    """! Actuator class which wraps around the Actuator C++ class """   
    def __init__(self, id: int, interface: USBInterface):
        """!
        Actuator class constructor
        
        @param id CAN Node ID of the Actuator
        @param interface Initialised CANInterface or USBInterface object
        """
        self.id = id
        interface = interface
        self.__actuator = nmotion_transport.Actuator(id, interface.getInterface())
    
    def getHardwareVersion(self) -> 'tuple[int, int, int, int, int]':
        """!
        Get the actuator hardware's current tag, major, minor and variant information.
        
        @return Tuple containing return status, tag, major, minor and variant information.
        """
        return self.__actuator.getHardwareVersion()

    def getFirmwareVersion(self) -> 'tuple[int, int, int, int]':
        """!
        Get the actuator firmware's major, minor and revision information.
        
        @return Tuple containing return status, major, minor and revision information.
        """
        return self.__actuator.getFirmwareVersion()

    def getFirmwareCommit(self) -> 'tuple[int, str]':
        """!
        Get the driver firmware's major, minor and revision information.
        
        @return Tuple containing return status, major, minor and revision information.
        """
        return self.__actuator.getFirmwareCommit()

    def emergencyStop(self) -> 'int':
        """!
        Function to issue an emergency stop to the device which stops the actuator 
        and sets the error flag; Since the error flag is set the actuator doesn't respond 
        to any other commands, until the actuator is rebooted.
        
        @return Return status
        """
        return self.__actuator.emergencyStop()
    
    def setDeviceToIdle(self) -> 'int':
        """!
        Set the actuator to idle state
        
        @return Return status
        """
        return self.__actuator.setDeviceToIdle()
    
    def setDeviceToActive(self) -> 'int':
        """!
        Set the actuator to active state
        
        @return Return status
        """
        return self.__actuator.setDeviceToActive()
    
    def setDCBusTripLevels(self, undervoltage_level: float, overvoltage_level: float) -> 'int':
        """!
        Set the actuator's undervoltage and overvoltage trip levels. The
        device will throw an error when the bus voltage will drop below
        under-voltage level or goes higher that the over-voltage level. Used to
        keep the actuator safe from high transient currents and voltage
        fluctuations.
        
        @param undervoltage_level under-voltage trip level
        @param overvoltage_level over-voltage trip level
        
        @return Return status
        """
        return self.__actuator.setDCBusTripLevels(undervoltage_level, overvoltage_level)
    
    def getDCBusTripLevels(self) -> 'tuple[int, float, float]':
        """!
        Get the actuator's current under-voltage and over-voltage levels.
        
        @return Tuple containing return status, undervoltage level & overvoltage level
        """
        return self.__actuator.getDCBusTripLevels()

    def setRegenCurrentTripLevel(self, regen_current_trip_level: float) -> int:
        """!
        Set the maximum regenerative current the power supply/battery can take.
        The device will throw DC_BUS_OVERREGEN Error if the current sink is more than this value
               
        @param regen_current_trip_level maximum regeneration current that battery/supply can take in
                
        @return Return status
        """
        return self.__actuator.setRegenCurrentTripLevel(regen_current_trip_level)

    def getRegenCurrentTripLevel(self) -> 'tuple[int, float]':
        """!
        Get the maximum regenerative current the power supply/battery can take.
        
        @return Tuple containing the return status and value of maximum regenerative current which is set.
        """
        return self.__actuator.getRegenCurrentTripLevel()

    def runCalibrationSequence(self) -> int:
        """!
        Run calibration sequence in the driver
        
        @return Return status
        """
        return self.__actuator.runCalibrationSequence()

    def setNodeId(self, id: int) -> 'int':
        """!
        Set the node id for the actuator
        
        @param id CAN Device ID of the actuator
        
        @return Return Status
        """
        return self.__actuator.setNodeId(id)

    def getNodeId(self) -> 'tuple[int, int]':
        """!
        Get current CAN Node ID of the actuator
        
        @return Tuple containing return status and the current CAN Node ID.
        """
        return self.__actuator.getNodeId()

    def enableMotorThermalLimit(self, upper_limit: int) -> int:
        """!
        Enable Motor based Thermal limit. A thermistor is used
        to measure motor temperature and start limiting the
        motor current once the temperature starts to approach the upper limit. The
        current limiting starts when the difference between the current temperature
        & the upper limit reaches 20 degrees celsius. The acutators will shut
        themselves off in case the temperature reaches the upper limit.
        
        @param upper_limit Shutdown temperature at which the actuator will stop.
        
        @return Return Status
        
        """
        return self.__actuator.enableMotorThermalLimit(upper_limit)
    
    def disableMotorThermalLimit(self, upper_limit: int) -> int:
        """!
        Disable Motor based thermal limit.
        
        @return Return Status
        """
        return self.__actuator.disableMotorThermalLimit()
    
    def getMotorThermistorConfiguration(self) -> 'tuple[bool, int, int, int, int]':
        """!
        Get the motor thermistor configuration
        
        @return Tuple containig return status, thermal limit enable status, upper limit and lower limit
        """
        return self.__actuator.getMotorThermistorConfiguration()
    
    def setPositionControllerGain(self, pos_gain: float) -> int:
        """!
        Set position controller gain value.
        
        @param pos_gain position gain value of the position controller
        
        @return Return Status
        """
        return self.__actuator.setPositionControllerGain(pos_gain)
    
    def getPositionControllerGain(self) -> 'tuple[int, float]':
        """!
        Get position controller gain value.
        
        @return Tuple containing return status and the current position controller gain value.
        """
        return self.__actuator.getPositionControllerGain()
    
    def setVelocityControllerGains(self, vel_gain:float, vel_integrator_gain: float) -> int:
        """!
        Set velocity controller gains.
        
        @param vel_gain velocity gain value of the velocity controller
        @param vel_integrator_gain velocity integrator gain value of the velocity controller
        
        @return Return status
        """
        return self.__actuator.setVelocityControllerGains(vel_gain, vel_integrator_gain)
    
    def getVelocityControllerGains(self) -> 'tuple[int, float, float]':
        """!
        Get controller velocity controller gains
        
        @return Tuple containing return status, velocity controller gain value and velocity controller integrator gain value.
        """
        return self.__actuator.getVelocityControllerGains()

    def setCurrentControllerBandwidth(self, bandwidth:float) -> int:
        """!
        Set current controller bandwith.
        
        @param bandwidth Bandwidth value for the current controller.
        
        @return Return status
        """
        return self.__actuator.setCurrentControllerBandwidth(bandwidth)

    def getCurrentControllerBandwidth(self) -> 'tuple[int, float]':
        """!
        Get current controller bandwidth.
        
        @return Tuple containing return status and the current controller bandwidth value.
        """
        return self.__actuator.getCurrentControllerBandwidth()
    
    def getCurrentControllerGains(self) -> 'tuple[int, float, float]':
        """!
        Get current controller gains.
        
        @return Tuple containing return status, current controller gain value and current controller integrator gain value.
        """
        return self.__actuator.getCurrentControllerGains()

    def setMotorEncoderBandwidth(self, bandwidth:float) -> int:
        """!
        Set motor encoder bandwidth value.
        
        @param bandwidth Bandwidth value
        
        @return Return status
        """
        return self.__actuator.setMotorEncoderBandwidth(bandwidth)

    def getMotorEncoderBandwidth(self) -> 'tuple[int, float]':
        """!
        Get current motor bandwidth value
        
        @return Tuple containing return status and current motor bandwidth value.
        """
        return self.__actuator.getMotorEncoderBandwidth()

    def setCurrentPostionToZero(self) -> int:
        """!
        Set current position of the actuator as zero.
        
        @return Return Statu
        """
        return self.__actuator.setCurrentPostionToZero()

    def getZeroPosition(self) -> 'tuple[int, float]':
        """!
        Get zero offset postion in degrees
        
        @return Tuple containing return status and the current zero offset position.
        """
        return self.__actuator.getZeroPosition()

    def setCurrentFilter(self, current_filter: float) -> int:
        """!
        Set the current data filter value in the controller
        
        @param current_filter Value of current filter.
        
        @return Return Status
        """
        return self.__actuator.setCurrentFilter(current_filter)

    def getCurrentFilter(self) -> 'tuple[float]':
        """!
        Get the current data filter value of the controller
        
        @return Tuple containing return status and the current data filter value.
        """
        return self.__actuator.getCurrentFilter()
    
    def setEncoderDataFilter(self, encoder_data_filter: float) -> int:
        """!
        Set the encoder data filter value in the controller
        
        @param encoder_data_filter value of encoder data filter.
        
        @return Return Status
        """        
        return self.__actuator.setEncoderDataFilter(encoder_data_filter)

    def getEncoderDataFilter(self) -> 'tuple[int, float]':
        """!
        Get the current value of encoder data filter in the controller.
        
        @return Tuple containing return status and the current encoder data filter value.
        """
        return self.__actuator.getEncoderDataFilter()

    def getOutputPosition(self) -> 'tuple[int, float]':
        """
        Get current angle of the output in degrees.
        
        @return Tuple containing return status and current output angle.
        """
        return self.__actuator.getOutputPosition()
    
    def getOutputVelocity(self) -> 'tuple[int, float]':
        """!
        Get current velocity of output in degrees per second.
        
        @return Tuple containing return status and current output velocity.
        """
        return self.__actuator.getOutputVelocity()
    
    def getOutputAcceleration(self) -> 'tuple[int, float]':
        """!
        Get current acceleration of the output in degrees per second^2
        
        @return Tuple containing return status and current output acceleration.
        """
        return self.__actuator.getOutputAcceleration()
    
    def getOutputTorque(self) -> 'tuple[int, float]':
        """!
        Get current torque at the output in Nm
        
        @return Tuple containing return status and output torque.
        """
        return self.__actuator.getOutputTorque()
    
    def getMotorPosition(self) -> 'tuple[int, float]':
        """!
        Get current angle of the motor rotor in number of rotations
        
        @return Tuple contatining return status and the current rotation value
        """
        return self.__actuator.getMotorPosition()
    
    def getMotorVelocity(self) -> 'tuple[int, float]':
        """!
        Get current velocity of the motor rotor in rotations per second
        
        @return Tuple containing return status and the current velocity value.
        """
        return self.__actuator.getMotorVelocity()
    
    def getMotorAcceleration(self) -> 'tuple[int, float]':
        """!
        Get current acceleration of the motor rotor in rotations per second^2
        
        @return Tuple containing return status and the current acceleration value.
        """
        return self.__actuator.getMotorAcceleration()
    
    def getMotorTorque(self) -> 'tuple[int, float]':
        """!
        Get torque of the motor rotor in Nm
        
        @return Tuple contatining return status and torque value.
        """
        return self.__actuator.getMotorTorque()
    
    def getMotorTemperature(self) -> 'tuple[int, float]':
        """!
        Get motor temperature in degree Celsius
        
        @return Tuple containing return status and motor temperature.
        """
        return self.__actuator.getMotorTemperature()
    
    def getDriverTemperature(self) -> 'tuple[int, float]':
        """!
        Get driver temperature in degree Celsius
        
        @return Tuple containing return status and driver temperature.
        """
        return self.__actuator.getDriverTemperature()

    def getMotorPhaseCurrents(self) -> 'tuple[int, float, float, float]':
        """!
        Get motor phase currents in Ampere.
        
        @return Tuple containing return status and currents in phase A, B and C 
        """
        return self.__actuator.getMotorPhaseCurrents()
    
    def getDCCalibPhaseCurrents(self) -> 'tuple[int, float, float, float]':
        """!
        Get DC Calib Phase Currents in Ampere.
        
        @return Tuple containing return status and currents in phase A, B and C 
        """
        return self.__actuator.getDCCalibPhaseCurrents()
    
    def getBusVoltage(self) -> 'tuple[int, float]':
        """!
        Get the bus voltage of the actuator
        
        @return Tuple containing return status and bus voltage.
        """        
        return self.__actuator.getBusVoltage()
    
    def getBusCurrent(self) -> 'tuple[int, float]':
        """!
        Get the bus current of the actuator
        
        @return Tuple containing return status and bus current.
        """
        return self.__actuator.getBusCurrent()
    
    def getIdqCurrents(self) -> 'tuple[int, float, float]':
        """!
        Get the Id and Iq currents of the motor
        
        @return Tuple containing return status, Id current and Iq current values.
        """
        return self.__actuator.getIdqCurrents()
    
    def getControllerState(self) -> 'tuple[int, int]':
        """!
        Get the controller state of the actuator
        
        @return Tuple containing the return status and the controller state value.
        """
        return self.__actuator.getControllerState()
    
    def getMotorState(self) -> 'tuple[int, bool, bool]':
        """!
        Get the motor state of the actuator
        
        @return Tuple containing the return status, calibration state status (true means motor is calibrated) 
                and the armed state (true means motor is currently armed) status of the motor.
        """
        return self.__actuator.getMotorState()
    
    def getMotorEncoderState(self) -> 'tuple[int, bool, bool]':
        """!
        Get the motor encoder state of the actuator
        
        @return Tuple containing the return status, index state (true means encoder found the index) and 
        ready sate (true means the encoder is ready) of the motor encoder.
        """
        
        return self.__actuator.getMotorEncoderState()
    
    def getOutputEncoderState(self) -> 'tuple[int, bool]':
        """!
        Get the output encoder state of the actuator
        
        @return Tuple containing return status and ready state (true means the encoder is ready) of the output encoder.
        """
        return self.__actuator.getOutputEncoderState()
    
    def getTrajectoryDoneStatus(self) -> 'tuple[int, bool]':
        """!
        Get the trajectory status of the actuator
        
        @return Tuple containing return status and the trajectory done status (true means the motion along the trajectory is complete)
        """
        return self.__actuator.getTrajectoryDoneStatus()
    
    def getMotorPhaseParameters(self) -> 'tuple[int, float, float]':
        """!
        Get motor phase currents in Ampere.
        
        @return Tuple containing return status and currents in phase A, B and C 
        """
        return self.__actuator.getMotorPhaseParameters()

    def getMotorEncoderRawData(self) -> 'tuple[int, int]':
        """!
        Get the raw encoder count from the motor
        
        @return Tuple containing return status and raw count value of the incremental encoder.
        """
        return self.__actuator.getMotorEncoderRawData()
    
    def getOutputEncoderRawData(self) -> 'tuple[int, int]':
        """!
        Get the raw data from output encoder
        
        @return Tuple containing the return status and raw data value of the absolute encoder.
        """
        return self.__actuator.getOutputEncoderRawData()
    
    def getDriverFault(self) -> 'tuple[int, int]':
        """!
        Get the Driver error status from the actuator.
        
        @return Tuple containing the return status and the driver error value.
        """
        return self.__actuator.getDriverFault()

    def getErrorCode(self) -> 'tuple[int, int]':
        """!
        Get the error code from the actuator
        
        @return Tuple containing the return status and the error code value.
        """
        return self.__actuator.getErrorCode()

    def getDebugErrorCode(self) -> 'tuple[int, int, int, int, int, int, int]':
        """!
        Get the error code from the actuator
        
        @return Tuple containing the return status and the error code value.
        """
        return self.__actuator.getDebugErrorCode()
    
    def clearActuatorErrors(self) -> int:
        """!
        Clear actuator errors
        
        @return Return Status
        """
        return self.__actuator.clearActuatorErrors()

    def getCANCommunicationStatus(self) -> 'tuple[int, bool, bool]':
        """!
        Get CAN communication status
        
        @return Tuple containing the return status, master connection status 
            (true means master connection is OK) and heartbeat receive status 
            (true means the device is still receiving heartbeat from master).
        """
        return self.__actuator.getCANCommunicationStatus()
    
    def getControllerMode(self) -> 'tuple[int, int]':
        """!
        Get controller mode
        
        @return Tuple containing the return status and the control mode value.
        """
        return self.__actuator.getControllerMode()
    
    def setPositionControl(self, angle: float, degrees_per_seconds: float) -> int:
        """!
        Set the actuator in position control mode.
        
        @param angle angle in degrees to where the actuator should move.
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        
        @return Return Status
        """
        return self.__actuator.setPositionControl(angle, degrees_per_seconds)

    def setVelocityControl(self, degrees_per_seconds: float) -> int:
        """!
        Set the actuator in Velocity Control mode
        
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        
        @return Return Status
        """
        return self.__actuator.setVelocityControl(degrees_per_seconds)
    
    def setTorqueControl(self, torque:float, degrees_per_second: float) -> int:
        """!
        Set the actuator in torque control mode
        
        @param torque: Motor Torque in Nm
        @param degrees_per_second: Velocity limit in degrees per second when the actuator is moving.
        
        @return Return Status
        """
        return self.__actuator.setTorqueControl(torque, degrees_per_second)

    def setPositionControlWithFeedForward(self, angle: float, velocity_ff: float, torque_ff: float) -> int:
        """!
        Set the actuator in position control mode with feedForward.
        
        @param angle angle in degrees to where the actuator should move.
        @param velocity_ff feedForward velocity in degrees per second.
        @param torque_ff feedForward torque  in Nm.
        
        @return Return Status
        """
        return self.__actuator.setPositionControlWithFeedForward(angle, velocity_ff, torque_ff)

    def setVelocityControlWithFeedForward(self, degrees_per_seconds: float, torque_ff: float) -> int:
        """!
        Set the actuator in velocity control mode with feedForward parameters
        
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        @param torque_ff feedForward torque in Nm.
        
        @return Return Status
        """
        return self.__actuator.setVelocityControlWithFeedForward(degrees_per_seconds, torque_ff)

    def setTrapezoidalTrajectoryControl(self, angle: float, degrees_per_seconds: float, accel_rate: float, decel_rate: float) -> int:
        """!
        Set the actuator in trajectory control mode.
        
        @param angle angle in degrees to where the actuator should move
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move
        @param accel_rate acceleration rate as a factor of velocity in degrees per second^2.
        @param decel_rate deacceleration rate as a factor of velocity in degrees per second^2.
        
        @return Return Status
        """
        return self.__actuator.setTrapezoidalTrajectoryControl(angle, degrees_per_seconds, accel_rate, decel_rate)

    def setScurveTrajectoryControl(self, angle: float, degrees_per_seconds: float, accel_rate: float, jerk_rate: float) -> int:
        """!
        Set the actuator in trajectory control mode.
        
        @param angle angle in degrees to where the actuator should move
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move
        @param accel_rate acceleration rate as a factor of velocity in degrees per second^2.
        @param jerk_rate jerk rate as a factor of acceleration in degrees per second^3.
        
        @return Return Status
        """
        return self.__actuator.setScurveTrajectoryControl(angle, degrees_per_seconds, accel_rate, jerk_rate)

    def setVelocityRampControl(self, degrees_per_seconds: float, ramp_rate_degrees_per_second: float) -> int:
        """!
        Set the actuator in Velocity Ramp Control mode
        
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        @param ramp_rate_degrees_per_second velocity ramp rate.
        
        @return Return Status
        """
        return self.__actuator.setVelocityRampControl(degrees_per_seconds, ramp_rate_degrees_per_second)

    def rebootActuator(self) -> int:
        """!
        Reboot the Actuator
        
        @return Return Status
        """
        return self.__actuator.rebootActuator()
    
    def saveConfigurations(self) -> int:
        """!
        Save the configuration to the actuator.
        
        @return Return Status
        """
        return self.__actuator.saveConfigurations()

    def eraseConfigurations(self) -> int:
        """!
        Erase configuration of the actuator.
        
        @return Return Status
        """
        return self.__actuator.eraseConfigurations()

    def isConnected(self) -> 'bool':
        """!
        Check if the driver is still connected
        
        @return Connection Status
        """
        return self.__actuator.isConnected()

    def flash(self, firmare_path: str, is_in_dfu_mode: bool = False) -> 'bool':
        """!
        flash the firmware to the actuator

        @param firmware_path path to which the firmware file is located
        
        @return Connection Status
        """
        return self.__actuator.flash(firmare_path, is_in_dfu_mode)
