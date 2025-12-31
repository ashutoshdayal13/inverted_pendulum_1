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

class Driver():   
    """! Driver class which wraps around the Driver C++ class """   

    def __init__(self, id: int = -1, interface: USBInterface = None):
        """!
        Driver class constructor
        
        @param id CAN Node ID of the Driver
        @param interface Initialised CANInterface or USBInterface object
        """
        if(not interface):
            raise Exception("An USBInterface object needs to be passed")

        if(id < 0):
            try:
                self.id = 0
                interface = interface
                self.__driver = nmotion_transport.Driver(interface.getInterface())
            except TypeError:
                raise Exception("Windows does not suppport using NMotion Components via native USB port. Use it's CAN Interface instead with NLink Adapter.")
        else:
            self.id = id
            interface = interface
            self.__driver = nmotion_transport.Driver(id, interface.getInterface())
    
    def getHardwareVersion(self) -> 'tuple[int, int, int, int, int]':
        """!
        Get the driver hardware's current tag, major, minor and variant information.
        
        @return Tuple containing return status, tag, major, minor and variant information.
        """
        return self.__driver.getHardwareVersion()

    def getFirmwareVersion(self) -> 'tuple[int, int, int, int]':
        """!
        Get the driver firmware's major, minor and revision information.
        
        @return Tuple containing return status, major, minor and revision information.
        """
        return self.__driver.getFirmwareVersion()
    
    def getFirmwareCommit(self) -> 'tuple[int, str]':
        """!
        Get the driver firmware's commit information.
        
        @return Tuple containing return status, and commit information.
        """
        return self.__driver.getFirmwareCommit()
    
    def emergencyStop(self) -> int:
        """!
        Function to issue an emergency stop to the device which stops the actuator 
        and sets the error flag; Since the error flag is set the actuator doesn't respond 
        to any other commands, until the actuator is rebooted.
        
        @return Return status
        """
        return self.__driver.emergencyStop()
    
    def setDeviceToIdle(self) -> int:
        """!
        Set the actuator to idle state
        
        @return Return status
        """
        return self.__driver.setDeviceToIdle()
    
    def setDeviceToActive(self) -> int:
        """!
        Set the actuator to active state
        
        @return Return status
        """
        return self.__driver.setDeviceToActive()
    
    def setDCBusTripLevels(self, undervoltage_level: float, overvoltage_level: float) -> int:
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
        return self.__driver.setDCBusTripLevels(undervoltage_level, overvoltage_level)
    
    def getDCBusTripLevels(self) -> 'tuple[int, float, float]':
        """!
        Get the actuator's current under-voltage and over-voltage levels.
        
        @return Tuple containing return status, undervoltage level & overvoltage level
        """
        return self.__driver.getDCBusTripLevels()

    def enableBrakeResistor(self, brake_resistance_value: float) -> int:
        """!
        Enable the brake resistor in the driver and configure it.
        
        @param brake_resistance_value brake resistance value
        
        @return Return status
        """
        return self.__driver.enableBrakeResistor(brake_resistance_value)
    
    def disableBrakeResistor(self) -> int:
        """!
        Disable brake resistor and configure it.
                
        @return Return status
        """
        return self.__driver.disableBrakeResistor()

    def getBrakeResistorState(self) -> 'tuple[int, bool, float, bool, bool]':
        """!
        Get current state of brake resistor
                
        @return Tuple containing return status, enable status of brake, value of the brake resistor, arm status of brake and the saturation status of the brake. 
        """
        return self.__driver.getBrakeResistorState()

    def setRegenCurrentTripLevel(self, regen_current_trip_level: float) -> int:
        """!
        Set the maximum regenerative current the power supply/battery can take.
        The device will throw DC_BUS_OVERREGEN Error if the current sink is more than this value
               
        @param regen_current_trip_level maximum regeneration current that battery/supply can take in
                
        @return Return status
        """
        return self.__driver.setRegenCurrentTripLevel(regen_current_trip_level)    

    def getRegenCurrentTripLevel(self) -> 'tuple[int, float]':
        """!
        Get the maximum regenerative current the power supply/battery can take.
        
        @return Tuple containing the return status and value of maximum regenerative current which is set.
        """
        return self.__driver.getRegenCurrentTripLevel()

    def setMotorParameters(self, pole_pairs: int, kV_rating: int, current_limit: float) -> int:
        """!
        Set parameters of the motor which is connected to the driver
        
        @param pole_pairs pole-pair value of the motor
        @param kV_rating kV rating value of the motor
        @param current_limit current limit value for the motor
        
        @return Return status
        """
        return self.__driver.setMotorParameters(pole_pairs, kV_rating, current_limit)

    def getMotorParameters(self) -> 'tuple[int, int, int, float]':
        """!
        Get current motor parameters
        
        @return Tuple containing the return status, pole pairs, kV rating and current limit value of the motor.
        """
        return self.__driver.getMotorParameters()

    def setMotorCalibrationParameters(self, motor_calib_voltage: float, motor_calib_current: float) -> int:
        """!
        Set Motor Calibration Parameters. These values are used to calculate
        the phase resistance & inductance values.
        
        @param motor_calib_voltage calibration voltage used to calibrate motor
        @param motor_calib_current calibration current used to calibrate motor
        
        @return Return status
        """
        return self.__driver.setMotorCalibrationParameters(motor_calib_voltage, motor_calib_current)
    
    def getMotorCalibrationParameters(self) -> 'tuple[int, float, float]':
        """!
        Get current Motor Calibration Parameters
        
        @return Tuple containing return status, motor calibration voltage and motor calibration current.
        """
        return self.__driver.getMotorCalibrationParameters()

    def configureMotorEncoder(self, encoder_type: int, resolution_param: int = 4096, use_index: bool = True) -> int:
        """!
        Configure motor side encoder. Set it to onboard absolute, external
        incremental or external absolute. Output side and motor side encoders can't
        be set to same.
        
        @param encoder_type Type of encoder used
        @param resolution_param value of the encoder in case of an incremental encoder or resolution value in 2^bits in case of absolute encoder (this parameter is ignored in the case of using onboard absolute encoder) resolution_param is set to 4096 by default.
        @param use_index Flag toset whether to use the index pin for incremental encoder or not. Valid only when an incremental encoder is used. use_index is set to true by default.
        
        @return Return Status
        """
        return self.__driver.configureMotorEncoder(encoder_type, resolution_param, use_index)
    
    def getMotorEncoderConfiguration(self) -> 'tuple[int, int, int, bool]':
        """!
        Get motor encoder configuration
        
        @return Tuple containing return status, encoder type, resolution parameter (CPR in case of incremental encoder or resolution in bits in case of absolute encoder) and use_index flag value.
        """
        return self.__driver.getMotorEncoderConfiguration()

    def setGearBoxParameters(self, gear_ratio:float) -> int:
        """!
        Set gear ratio (Ouput / Input) of the Gearbox.
        By default the value is 1 in the driver.
        
        @param gear_ratio Gear ratio of the gearbox
        
        @return Return status
        """
        return self.__driver.setGearBoxParameters(gear_ratio)
    
    def getGearBoxParameters(self) -> 'tuple[int, float]':
        """!
        Get Gear ratio of the Gearbox
        
        @return Tuple containing return status and current gearbox ratio.
        """
        return self.__driver.getGearBoxParameters()

    def configureOutputEncoder(self, encoder_type: int, resolution_param: int = 4096, use_index: bool = True) -> int:
        """!
        Configure Output side encoder. Set it to onboard absolute, external
        incremental or external absolute. Output side and motor side encoders can't
        be set to same.
        
        @param encoder_type Type of encoder used
        @param resolution_param CPR value of the encoder in case of an incremental encoder or resolution value in 2^bits in case of absolute encoder (this parameter is ignored in the case of using onboard absolute encoder) resolution_param is set to 4096 by default.
        @param use_index Flag toset whether to use the index pin for incremental encoder or not. Valid only when an incremental encoder is used. use_index is set to true by default.
        
        @return Return Status
        """
        return self.__driver.configureOutputEncoder(encoder_type, resolution_param, use_index)

    def disableOutputEncoder(self) -> int:
        """!
        Disable Output Encoder.
        
        @return Return status
        """
        return self.__driver.disableOutputEncoder()
    
    def getOutputEncoderConfiguration(self) -> 'tuple[int, int, int, bool]':
        """!
        Get Output encoder configuration.
        
        @return Tuple containing return status, encoder type, resolution parameter (CPR in case of incremental encoder or resolution in bits in case of absolute encoder) and use_index flag value.
        """
        return self.__driver.getOutputEncoderConfiguration()
        
    def runEncoderIndexSearch(self) -> int:
        """!
        Run encoder index search sequence in the driver
        
        @return Return status
        """
        return self.__driver.runEncoderIndexSearch()
    
    def runEncoderMatching(self) -> int:
        """!
        Run matching motor encoder with output encoder in the driver
        
        @return Return status
        """
        return self.__driver.runEncoderMatching()

    def runCalibrationSequence(self) -> int:
        """!
        Run calibration sequence in the driver
        
        @return Return status
        """
        return self.__driver.runCalibrationSequence()

    def runEncoderCalibration(self) -> int:
        """!
        Run encoder calibration in the driver
        
        @return Return status
        """
        return self.__driver.runEncoderCalibration()

    def setNodeId(self, id:int) -> int:
        """!
        Set the node id for the actuator
        
        @param id CAN Device ID of the actuator
        
        @return Return Status
        """
        return self.__driver.setNodeId(id)

    def getNodeId(self) -> 'tuple[int, int]':
        """!
        Get current CAN Node ID of the actuator
        
        @return Tuple containing return status and the current CAN Node ID.
        """
        return self.__driver.getNodeId()
    
    def setStartupConfigs(self, index_search:bool = False, encoder_matching:bool = False, closed_loop:bool = False) -> int:
        return self.__driver.setStartupConfigs(index_search, encoder_matching, closed_loop)

    def getStartupConfigs(self) -> 'tuple[int, bool, bool, bool]':
        return self.__driver.getStartupConfigs()

    def setMotorThermistorParameters(self, r_ref: float, beta: int) -> int:
        """!
        Set Motor thermistor parameters
        @param r_ref Reference Resistance at 25 degrees celsius of the NTC themistor
        @param beta Beta value of the thermistor
        
        @return Return status.
        """
        return self.__driver.setMotorThermistorParameters(r_ref, beta)

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
        return self.__driver.enableMotorThermalLimit(upper_limit)
    
    def disableMotorThermalLimit(self) -> int:
        """!
        Disable Motor based thermal limit.
        
        @return Return Status
        """
        return self.__driver.disableMotorThermalLimit()
    
    def getMotorThermistorConfiguration(self) -> 'tuple[int, bool, float, int, int, int]':
        """!
        Get the motor thermistor configuration
        
        @return Tuple containig return status, thermal limit enable status, upper limit and lower limit
        """
        return self.__driver.getMotorThermistorConfiguration()
    
    def setTorqueLimit(self, torque_limit: float) -> int:
        """!
        Set Torque Limit for the driver.
        
        @param torque_limit Torque limit value in Nm.
        
        @return Return Status
        """
        return self.__driver.setTorqueLimit(torque_limit)
        
    def getTorqueLimit(self) -> 'tuple[int, float]':
        """!
        Get the controller torque limit
        
        @return  Tuple containing return status and the torque limit of the controller
        """
        return self.__driver.getTorqueLimit()
    
    def setPositionControllerGain(self, pos_gain: float) -> int:
        """!
        Set position controller gain value.
        
        @param  Tuple containing return status and pos_gain position gain value of the position controller
        
        @return Return Status
        """
        return self.__driver.setPositionControllerGain(pos_gain)
    
    def getPositionControllerGain(self) -> 'tuple[int, float]':
        """!
        Get position controller gain value.
        
        @return Tuple containing return status and the current position controller gain value.
        """
        return self.__driver.getPositionControllerGain()
    
    def setVelocityControllerGains(self, vel_gain:float, vel_integrator_gain: float) -> int:
        """!
        Set velocity controller gains.
        
        @param vel_gain velocity gain value of the velocity controller
        @param vel_integrator_gain velocity integrator gain value of the velocity controller
        
        @return Return status
        """
        return self.__driver.setVelocityControllerGains(vel_gain, vel_integrator_gain)
    
    def getVelocityControllerGains(self) -> 'tuple[int, float, float]':
        """!
        Get controller velocity controller gains
        
        @return Tuple containing return status, velocity controller gain value and velocity controller integrator gain value.
        """
        return self.__driver.getVelocityControllerGains()

    def setCurrentControllerBandwidth(self, bandwidth:float) -> int:
        """!
        Set current controller bandwith.
        
        @param bandwidth Bandwidth value for the current controller.
        
        @return Return status
        """
        return self.__driver.setCurrentControllerBandwidth(bandwidth)

    def getCurrentControllerBandwidth(self) -> 'tuple[int, float]':
        """!
        Get current controller bandwidth.
        
        @return Tuple containing return status and the current controller bandwidth value.
        """
        return self.__driver.getCurrentControllerBandwidth()
    
    def getCurrentControllerGains(self) -> 'tuple[int, float, float]':
        """!
        Get current controller gains.
        
        @return Tuple containing return status, current controller gain value and current controller integrator gain value.
        """
        return self.__driver.getCurrentControllerGains()

    def setMotorEncoderBandwidth(self, bandwidth:float) -> int:
        """!
        Set motor encoder bandwidth value.
        
        @param bandwidth Bandwidth value
        
        @return Return status
        """
        return self.__driver.setMotorEncoderBandwidth(bandwidth)

    def getMotorEncoderBandwidth(self) -> 'tuple[int, float]':
        """!
        Get current motor bandwidth value
        
        @return Tuple containing return status and current motor bandwidth value.
        """
        return self.__driver.getMotorEncoderBandwidth()

    def setCurrentPostionToZero(self) -> int:
        """!
        Set current position of the actuator as zero.
        
        @return Return Statu
        """
        return self.__driver.setCurrentPostionToZero()

    def getZeroPosition(self) -> 'tuple[int, float]':
        """!
        Get zero offset postion in degrees
        
        @return Tuple containing return status and the current zero offset position.
        """
        return self.__driver.getZeroPosition()

    def setMotorPhaseParameters(self, phase_resistance:float, phase_inductance:float) -> int:
        """!
        Set motor phase parameters.
        
        @param phase_resistance Phase resistance of the motor
        @param phase_inductance Phase inductance of the motor
        
        @return Return status
        """
        return self.__driver.setMotorPhaseParameters(phase_resistance, phase_inductance)

    def setCurrentFilter(self, current_filter: float) -> int:
        """!
        Set the current data filter value in the controller
        
        @param current_filter Value of current filter.
        
        @return Return Status
        """
        return self.__driver.setCurrentFilter(current_filter)

    def getCurrentFilter(self) -> 'tuple[int, float]':
        """!
        Get the current data filter value of the controller
        
        @return Tuple containing return status and the current data filter value.
        """
        return self.__driver.getCurrentFilter()
    
    def setEncoderDataFilter(self, encoder_data_filrer: float) -> int:
        """!
        Set the encoder data filter value in the controller
        
        @param encoder_data_filter value of encoder data filter.
        
        @return Return Status
        """        
        return self.__driver.setEncoderDataFilter(encoder_data_filrer)

    def getEncoderDataFilter(self) -> 'tuple[int, float]':
        """!
        Get the current value of encoder data filter in the controller.
        
        @return Tuple containing return status and the current encoder data filter value.
        """
        return self.__driver.getEncoderDataFilter()
    
    def setMAxxxBCTValue(self, encoder:bool, bct_value: int) -> int:
        """!
        Set the encoder bct value of maxxx encoder
        
        @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
        @param bct_value BCT value to be set in the controller
        
        @return Return Status
        """        
        return self.__driver.setMAxxxBCTValue(encoder, bct_value)

    def getMAxxxBCTValue(self, encoder:bool) -> 'tuple[int, int, bool]':
        """!
        Get the encoder bct value of the maxxx encoder.
        
        @return Tuple containing return status and the current encoder bct value value and the spi read status.
        """
        return self.__driver.getMAxxxBCTValue(encoder)

    def setMAxxxTrimmingSettings(self, encoder:bool, etx: bool, ety: bool) -> int:
        """!
        Set the encoder data filter value in the controller
        
        @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
        @param etx set the encoder trimming value for x axis
        @param ety set the encoder trimming value for y axis
        
        @return Return Status
        """        
        return self.__driver.setMAxxxTrimmingSettings(encoder, etx, ety)

    def getMAxxxTrimmingSettings(self, encoder:bool) -> 'tuple[int, bool, bool, bool]':
        """!
        Get the current value of encoder bct value in the controller.

        @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
        @param etx get the encoder trimming value for x axis
        @param ety get the encoder trimming value for y axis
        @param spi_read_status get the spi read status
        
        @return Tuple containing return status and the current encoder bct value value.
        """
        return self.__driver.getMAxxxTrimmingSettings(encoder)

    def setMAxxxFilterWindow(self, encoder:bool, filter_window: int) -> int:
        """!
        Set the encoder data filter value in the controller
        
        @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
        @param filter_window Filter window value to be set in the controller
        
        @return Return Status
        """        
        return self.__driver.setMAxxxFilterWindow(encoder, filter_window)

    def getMAxxxFilterWindow(self, encoder:bool) -> 'tuple[int, int, bool]':
        """!
        Get the current value of encoder bct value in the controller.

        @param encoder Flag to set the encoder type. False for motor side encoder True for output side encoder
        @param filter_window get the encoder filter window value
        @param spi_read_status get the spi read status
                
        @return Tuple containing return status and the current encoder bct value value.
        """
        return self.__driver.getMAxxxFilterWindow(encoder)

    def getOutputPosition(self) -> 'tuple[int, float]':
        """
        Get current angle of the output in degrees.
        
        @return Tuple containing return status and current output angle.
        """
        return self.__driver.getOutputPosition()
    
    def getOutputVelocity(self) -> 'tuple[int, float]':
        """!
        Get current velocity of output in degrees per second.
        
        @return Tuple containing return status and current output velocity.
        """
        return self.__driver.getOutputVelocity()
    
    def getOutputAcceleration(self) -> 'tuple[int, float]':
        """!
        Get current acceleration of the output in degrees per second^2
        
        @return Tuple containing return status and current output acceleration.
        """
        return self.__driver.getOutputAcceleration()
    
    def getOutputTorque(self) -> 'tuple[int, float]':
        """!
        Get current torque at the output in Nm
        
        @return Tuple containing return status and output torque.
        """
        return self.__driver.getOutputTorque()
    
    def getMotorPosition(self) -> 'tuple[int, float]':
        """!
        Get current angle of the motor rotor in number of rotations
        
        @return Tuple contatining return status and the current rotation value
        """
        return self.__driver.getMotorPosition()
    
    def getMotorVelocity(self) -> 'tuple[int, float]':
        """!
        Get current velocity of the motor rotor in rotations per second
        
        @return Tuple containing return status and the current velocity value.
        """
        return self.__driver.getMotorVelocity()
    
    def getMotorAcceleration(self) -> 'tuple[int, float]':
        """!
        Get current acceleration of the motor rotor in rotations per second^2
        
        @return Tuple containing return status and the current acceleration value.
        """
        return self.__driver.getMotorAcceleration()
    
    def getMotorTorque(self) -> 'tuple[int, float]':
        """!
        Get torque of the motor rotor in Nm
        
        @return Tuple contatining return status and torque value.
        """
        return self.__driver.getMotorTorque()
    
    def getMotorTemperature(self) -> 'tuple[int, float]':
        """!
        Get motor temperature in degree Celsius
        
        @return Tuple containing return status and driver temperature.
        """
        return self.__driver.getMotorTemperature()
    
    def getDriverTemperature(self) -> 'tuple[int, float]':
        return self.__driver.getDriverTemperature()
    
    def getMotorPhaseCurrents(self) -> 'tuple[int, float]':
        """!
        Get motor phase currents in Ampere.
        
        @return Tuple containing return status and currents in phase A, B and C 
        """
        return self.__driver.getMotorPhaseCurrents()

    def getDCCalibPhaseCurrents(self) -> 'tuple[int, float, float, float]':
        """!
        Get DC Calib Phase Currents in Ampere.
        
        @return Tuple containing return status and currents in phase A, B and C 
        """
        return self.__driver.getDCCalibPhaseCurrents()
    
    def getBusVoltage(self) -> 'tuple[int, float]':
        """!
        Get the bus voltage of the actuator
        
        @return Tuple containing return status and bus voltage.
        """       
        return self.__driver.getBusVoltage()
    
    def getBusCurrent(self) -> 'tuple[int, float]':
        """!
        Get the bus current of the actuator
        
        @return Tuple containing return status and bus current.
        """
        return self.__driver.getBusCurrent()
    
    def getIdqCurrents(self) -> 'tuple[int, float, float]':
        """!
        Get the Id and Iq currents of the motor
        
        @return Tuple containing return status, Id current and Iq current values.
        """
        return self.__driver.getIdqCurrents()
    
    def getBrakeCurrent(self) -> 'tuple[int, float]':
        """!
        Get the value of current passing through the brake.
        
        @return Tuple containing return status and brake current.
        """
        return self.__driver.getBrakeCurrent()

    
    def getControllerState(self) -> 'tuple[int, int]':
        """!
        Get the controller state of the actuator
        
        @return Tuple containing the return status and the controller state value.
        """
        return self.__driver.getControllerState()
    
    def getMotorState(self) -> 'tuple[int, bool, bool]':
        """!
        Get the motor state of the actuator
        
        @return Tuple containing the return status, calibration state status (true means motor is calibrated) 
                and the armed state (true means motor is currently armed) status of the motor.
        """
        return self.__driver.getMotorState()
    
    def getMotorEncoderState(self) -> 'tuple[int, bool, bool]':
        """!
        Get the motor encoder state of the actuator
        
        @return Tuple containing the return status, index state (true means encoder found the index) and 
        ready sate (true means the encoder is ready) of the motor encoder.
        """
        return self.__driver.getMotorEncoderState()
    
    def getOutputEncoderState(self) -> 'tuple[int, bool, bool]':
        """!
        Get the output encoder state of the actuator
        
        @return Tuple containing return status, index state (true means encoder found the index) and ready state (true means the encoder is ready) of the output encoder.
        """
        return self.__driver.getOutputEncoderState()
    
    def getTrajectoryDoneStatus(self) -> 'tuple[int, bool]':
        """!
        Get the trajectory status of the actuator
        
        @return Tuple containing return status and the trajectory done status (true means the motion along the trajectory is complete)
        """
        return self.__driver.getTrajectoryDoneStatus()

    def getMotorPhaseParameters(self) -> 'tuple[int, float, float]':
        """!
        Get motor phase currents in Ampere.
        
        @return Tuple containing return status and currents in phase A, B and C 
        """
        return self.__driver.getMotorPhaseParameters()
    
    def getMotorEncoderRawData(self) -> 'tuple[int, int]':
        """!
        Get the raw data from motor encoder
        
        @return Tuple containing the return status and raw data value of the motor encoder.
        """
        return self.__driver.getMotorEncoderRawData()
    
    def getOutputEncoderRawData(self) -> 'tuple[int, int]':
        """!
        Get the raw data from output encoder
        
        @return Tuple containing the return status and raw data value of the absolute encoder.
        """
        return self.__driver.getOutputEncoderRawData()
    
    def getDriverFault(self) -> 'tuple[int, int]':
        """!
        Get the Driver error status from the actuator.
        
        @return Tuple containing the return status and the driver error value.
        """
        return self.__driver.getDriverFault()

    def getErrorCode(self) -> 'tuple[int, int]':
        """!
        Get the error code from the driver
        
        @return Tuple containing the return status and the error code value.
        """
        return self.__driver.getErrorCode()

    def getDebugErrorCode(self) -> 'tuple[int, int, int, int, int, int, int]':
        """!
        Get the debug error codes for errors from the driver
        
        @return Tuple containing the return status and the error code value.
        """
        return self.__driver.getDebugErrorCode()
    
    def clearDriverErrors(self) -> int:
        """!
        Get the error code from the driver
        
        @return Tuple containing the return status and the error code value.
        """
        return self.__driver.clearDriverErrors()

    def getCANCommunicationStatus(self) -> 'tuple[int, bool, bool]':
        """!
        Get CAN communication status
        
        @return Tuple containing the return status, master connection status 
            (true means master connection is OK) and heartbeat receive status 
            (true means the device is still receiving heartbeat from master).
        """
        return self.__driver.getCANCommunicationStatus()
    
    def getControllerMode(self) -> 'tuple[int, int]':
        """!
        Get controller mode
        
        @return Tuple containing the return status and the control mode value.
        """        
        return self.__driver.getControllerMode()
    
    def setPositionControl(self, angle: float, degrees_per_seconds: float) -> int:
        """!
        Set the driver in position control mode.
        
        @param angle angle in degrees to where the actuator should move.
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        
        @return Return Status
        """ 
        return self.__driver.setPositionControl(angle, degrees_per_seconds)
    
    def setVelocityControl(self, degrees_per_seconds: float) -> int:
        """!
        Set the driver in Velocity Control mode
        
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        
        @return Return Status
        """
        return self.__driver.setVelocityControl(degrees_per_seconds)
    
    def setTorqueControl(self, torque:float, degrees_per_second: float) -> int:
        """!
        Set the Driver in torque control mode
        
        @param torque: Motor Torque in Nm
        @param degrees_per_second: Velocity limit in degrees per second when the actuator is moving.
        
        @return Return Status
        """
        return self.__driver.setTorqueControl(torque, degrees_per_second)

    def setPositionControlWithFeedForward(self, angle: float, velocity_ff: float, torque_ff: float) -> int:
        """!
        Set the actuator in position control mode with feedfordword parameters.
        
        @param angle angle in degrees to where the actuator should move.
        @param velocity_ff feedForward velocity in degrees per second.
        @param torque_ff feedForward torque  in Nm.
        
        @return Return Status
        """
        return self.__driver.setPositionControlWithFeedForward(angle, velocity_ff, torque_ff)

    def setVelocityControlWithFeedForward(self, degrees_per_seconds: float, torque_ff: float) -> int:
        """!
        Set the actuator in velocity control mode with feedForward parameters
        
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        @param torque_ff feedForward torque in Nm.
        
        @return Return Status
        """
        return self.__driver.setVelocityControlWithFeedForward(degrees_per_seconds, torque_ff)

    def setTrapezoidalTrajectoryControl(self, angle: float, degrees_per_seconds: float, accel_rate: float, decel_rate: float) -> int:
        """!
        Set the actuator in Trapezoidal trajectory control mode.
        
        @param angle angle in degrees to where the actuator should move
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move
        @param accel_rate acceleration rate as a factor of velocity in degrees per second^2.
        @param decel_rate deacceleration rate as a factor of velocity in degrees per second^2.
        
        @return Return Status
        """
        return self.__driver.setTrapezoidalTrajectoryControl(angle, degrees_per_seconds, accel_rate, decel_rate)

    def setScurveTrajectoryControl(self, angle: float, degrees_per_seconds: float, accel_rate: float, jerk_rate: float) -> int:
        """!
        Set the actuator in S-Curve trajectory control mode.
        
        @param angle angle in degrees to where the actuator should move
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move
        @param accel_rate acceleration rate as a factor of velocity in degrees per second^2.
        @param jerk_rate jerk rate as a factor of acceleration in degrees per second^3.
        
        @return Return Status
        """
        return self.__driver.setScurveTrajectoryControl(angle, degrees_per_seconds, accel_rate, jerk_rate)

    def setVelocityRampControl(self, degrees_per_seconds: float, ramp_rate_degrees_per_second: float) -> int:
        """!
        Set the actuator in Velocity Ramp Control mode
        
        @param degrees_per_seconds velocity in degrees per second at which the actuator should move.
        @param ramp_rate_degrees_per_second velocity ramp rate.
        
        @return Return Status
        """
        return self.__driver.setVelocityRampControl(degrees_per_seconds, ramp_rate_degrees_per_second)

    def rebootDriver(self) -> int:
        """!
        Reboot the Driver.
        
        @return Return Status
        """
        return self.__driver.rebootDriver()
    
    def saveConfigurations(self) -> int:
        """!
        Save the configuration to the actuator.
        
        @return Return Status
        """
        return self.__driver.saveConfigurations()

    def eraseConfigurations(self) -> int:
        """!
        Erase configuration of the actuator.
        
        @return Return Status
        """
        return self.__driver.eraseConfigurations()

    def enterDFUMode(self) -> int:
        """!
        Set driver to DFU mode
        
        @return Return Status
        """
        return self.__driver.enterDFUMode()
    
    def isConnected(self) -> 'bool':
        """!
        Check if the driver is still connected
        
        @return Connection Status
        """
        return self.__driver.isConnected()
    
    def flash(self, firmare_path: str, is_in_dfu_mode: bool = False) -> 'bool':
        """!
        flash the firmware to the driver

        @param firmware_path path to which the firmware file is located
        
        @return Connection Status
        """
        return self.__driver.flash(firmare_path, is_in_dfu_mode)
