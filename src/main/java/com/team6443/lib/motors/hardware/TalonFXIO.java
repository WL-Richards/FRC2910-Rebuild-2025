// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.motors.hardware;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.DynamicMotionMagicVoltage;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.can.interfaces.CANable;
import com.team6443.lib.config.motors.ServoMotorConfiguration;
import com.team6443.lib.factories.motors.TalonFXFactory;
import com.team6443.lib.motors.MotorInputs;
import com.team6443.lib.motors.interfaces.MotorIO;
import com.team6443.lib.phoenix6.CTREUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** 
 * Generic implementation for the Talon FX
*/
public class TalonFXIO implements MotorIO, CANable{
    protected final TalonFX talon;

    private final ServoMotorConfiguration<TalonFXConfiguration> config;

    // Object to drive output using a duty cycle control 
    private final DutyCycleOut dutyCycleControl = new DutyCycleOut(0.0);

    // Target velocity controller using voltage PID and FF to achieve the desired result
    private final VelocityVoltage velocityVoltageControl = new VelocityVoltage(0.0);

    // Target velocity controller using motion magic to achieve the desired velocity
    private final MotionMagicVelocityVoltage motionMagicVelocityControl = new MotionMagicVelocityVoltage(0.0);

    // Target position using voltage PID and FF to achieve the deisred position
    private final PositionVoltage positionVoltageControl = new PositionVoltage(0.0);

    // Direct voltage controller that will attempt to drive the motor at a set voltage
    private final VoltageOut voltageControl = new VoltageOut(0.0);

    // Request that motion magic drive motor voltage to reach a position setpoint using a preconstructed motion profiles
    private final MotionMagicVoltage motionMagicPositionControl = new MotionMagicVoltage(0.0);

    // Request that motion magic drive motor voltage to reach a position setpoint. Allowing for real time motion profiling 
    private final DynamicMotionMagicVoltage dynamicMotionMagicPositionControl = new DynamicMotionMagicVoltage(
        0.0, 
        0.0, 
        0.0, 
        0.0
    );

    // Controller used to mimic the inputs of another TalonFX
    private final Follower followerControl = new Follower(0, true);

    // Drive a motor to a desired stator current
    private final TorqueCurrentFOC torqueCurrentFOCControl = new TorqueCurrentFOC(0.0);

    // Signal used to retreive current position values from the motor, this could be the position of a remote sensor that we are using as feedback
    private final StatusSignal<Angle> positionSignal;

    // Signal used to retreive current velocity values from the motor
    private final StatusSignal<AngularVelocity> velocitySignal;

    // Signal used to retreive current voltage values from the motor
    private final StatusSignal<Voltage> voltageSignal;

    // Signal used to retreive the current stator current applied to the motor
    private final StatusSignal<Current> currentStatorSignal;

    // Signal used to retrieve the current supply current applied to the motor
    private final StatusSignal<Current> currentSupplySignal;

    // Signal used to retrieve the current rotor position of the motor (likely the same as positionSignal unless using remote sensors)
    private final StatusSignal<Angle> rotorPositionSignal;

    // All the status signals we are using for this TalonFX
    private final BaseStatusSignal[] signals;

    private TalonFXIO(
        CANDeviceID device, 
        TalonFX talonFX,
        TalonFXConfiguration motorConfig, 
        ServoMotorConfiguration<TalonFXConfiguration> servoMotorConfig
    ){

        // If an actual talon is passed in
        if (talonFX != null){
            this.config = new ServoMotorConfiguration<TalonFXConfiguration>();
            talon = talonFX;
        }
        // If servo motor configuration is supplied use that to create the motor
        else if(servoMotorConfig != null){
            this.config = servoMotorConfig;
            talon = TalonFXFactory.createRawWithConfig(device, servoMotorConfig.getMotorConfig());
        }

        // If raw motor configuration then create new servo motor configuration supplying our motor config
        else if(motorConfig != null){
            this.config = new ServoMotorConfiguration<TalonFXConfiguration>().withConfig(motorConfig);
            talon = TalonFXFactory.createRawWithConfig(device, motorConfig);
        }

        // This is kinda undefined case, so we throw some errors, it will still work as it creates a talon as expected but default configuration is used
        else{
            this.config = new ServoMotorConfiguration<TalonFXConfiguration>().withConfig(TalonFXFactory.getDefaultConfig());
            talon = TalonFXFactory.createRawDefault(device);
            DriverStation.reportError("!!!!! Talon FX IO Created with no Explicit ServoMotorConfiguration or TalonFXConfiguration !!!!!", false);
        }
        

        // Set signal sources
        positionSignal = talon.getPosition();
        velocitySignal = talon.getVelocity();
        voltageSignal = talon.getMotorVoltage();
        currentStatorSignal = talon.getStatorCurrent();
        currentSupplySignal = talon.getSupplyCurrent();
        rotorPositionSignal = talon.getRotorPosition();

        // Setup list of signals to update all at once
        signals = new BaseStatusSignal[]{
            positionSignal,
            velocitySignal,
            voltageSignal,
            currentStatorSignal,
            currentSupplySignal,
            rotorPositionSignal
        };

        // Set the update frequency for all the signals on this device
        CTREUtil.setUpdateFrequencyForAll(
            50.0, 
            signals, 
            talon.getDeviceID()
        );

        // Optimization the bus utilization for the talon
        CTREUtil.Configuration.Motors.optimizeBusUtilization(talon);
    }

    /**
     * Construct a new instance of the TalonFXIO device with a ServoMotorConfiguration
     * @param device The CAN device that represents this motor
     * @param servoMotorConfig The configuration used to determine how the motor should be driven outside the context of just the motor
     */
    public TalonFXIO(CANDeviceID device, ServoMotorConfiguration<TalonFXConfiguration> servoMotorConfig){
        this(device, null, null, servoMotorConfig);
        
    }

    /**
     * Construct a new instance of the TalonFXIO device with a TalonFX configuration
     * @param device The CAN device that represents this motor
     * @param motorConfiguration The TalonFXConfiguration config used to determine how the motor should be driven
     */
    public TalonFXIO(CANDeviceID device, TalonFXConfiguration motorConfiguration){
        this(device, null, motorConfiguration, null);
    }

    /**
     * Construct a new instance of the TalonFXIO device with a TalonFX configuration
     * @param device The CAN device that represents this motor
     * @param motor The TalonFX to use with this wrapper
     */
    public TalonFXIO(CANDeviceID device, TalonFX motor){
        this(device, motor, null, null);
    }

    /**
     * Get a reference to underlying motor itself
     * @return The underlying talon object that we are wrapping
     */
    public TalonFX getTalon(){
        return talon;
    }

    /**
     * Get the name of this given motor
     */
    @Override
    public String getName(){
        return getCANDeviceName();
    }

    /**
     * Convert the current rotor rotations to the real-world units specified scaled by the unitToRotorRotationRation defined in the config
     * @param rotorRotations Rotor rotation value we want to convert into the in-use units
     * @return The rotations converted into some units as defined in the config
     */
    private double getRotorRotationsToUnits(double rotorRotations){
        return this.config.getRotorRotationsToUnits(rotorRotations);
    }

    /**
     * Convert the current units into rotor rotations using the defined conversion ratio
     * @param units Units we want to convert to rotor rotations
     * @return The resulting rotor rotations 
     */
    public double getUnitsToRotorRotations(double units){
        return this.config.getUnitsToRotorRotations(units);
    }

    /**
     * Given some value within our defined unit space we want to clamp it within our defined min and max and then convert it to a rotor position
     * @param units Units we want to convert to a clampped rotor position
     * @return Clamped rotor rotation
     */
    private double clampPosition(double units){
        return getUnitsToRotorRotations(
            MathUtil.clamp(units, this.config.kMinPositionUnits, this.config.kMaxPositionUnits)
        );
    }

    /**
     * Update the current motor state as reported by the motor itself
     * 
     * @param inputs Inputs that are to be updated by the current state of this motor
     */
    @Override
    public boolean updateInputs(MotorInputs inputs) {
        StatusCode refreshStatus = BaseStatusSignal.refreshAll(signals);

        inputs.unitPosition = getRotorRotationsToUnits(positionSignal.getValueAsDouble());
        inputs.velocityUnitsPerSecond = getRotorRotationsToUnits(velocitySignal.getValueAsDouble());
        inputs.appliedVolts = voltageSignal.getValueAsDouble();
        inputs.currentStatorAmps = currentStatorSignal.getValueAsDouble();
        inputs.currentSupplyAmps = currentSupplySignal.getValueAsDouble();
        inputs.rawRotorPosition = rotorPositionSignal.getValueAsDouble();

        return refreshStatus == StatusCode.OK;
    }

    /**
     * Get the CAN device this TalonFX is based on
     */
    @Override
    public CANDeviceID getCANDevice() {
        return this.config.kCANDevice;
    }

    // ------- Motor Configuration -------

    /**
     * Set the neutral state of the motor (ie should we lock up when no input is provided (brake) or move freely (coast))
     * @param mode Library agnostic representation of the motor neutral state
     */
    @Override
    public boolean setNeutralMode(NeutralMode mode) {
        // Update the motor configuration
        switch(mode){
            case BRAKE:
                this.config.getMotorConfig().MotorOutput.NeutralMode = NeutralModeValue.Brake;
                break;
            case COAST:
                this.config.getMotorConfig().MotorOutput.NeutralMode = NeutralModeValue.Coast;
                break;
        }

        // Apply the updated configuration
        return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
    }

    /**
     *  Enable the software defined rotation limits for this talon 
     * 
     *  @param forwardLimitEnabled Enable the software limit for the forward direction of the motor
     *  @param reverseLimitEnabled Enable the software limit for the reverse direction of the motor 
     */
    @Override
    public boolean setEnableSoftwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
        this.config.getMotorConfig().SoftwareLimitSwitch.ForwardSoftLimitEnable = forwardLimitEnabled;
        this.config.getMotorConfig().SoftwareLimitSwitch.ReverseSoftLimitEnable = reverseLimitEnabled;
        return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
    }

    /**
     * Get the current enable state of the software limits
     * @return A pair of booleans that store the current state of the software limits in both forward and reverse directions
     */
    @Override
    public Pair<Boolean, Boolean> getEnableSoftwareLimits() {
        return new Pair<Boolean,Boolean>(
            this.config.getMotorConfig().SoftwareLimitSwitch.ForwardSoftLimitEnable, 
            this.config.getMotorConfig().SoftwareLimitSwitch.ReverseSoftLimitEnable
        );
    }

    /**
     *  Enable the hardware defined rotation limits for this Talon 
     * 
     *  @param forwardLimitEnabled Enable the hardware limit for the forward direction of the motor
     *  @param reverseLimitEnabled Enable the hardware limit for the reverse direction of the motor 
     */
    @Override
    public boolean setEnableHardwareLimits(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
        this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnabled;
        this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnabled;
        return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
    }

    /**
     *  Enable the hardware defined rotation limits for this Talon and enable autozeroing if the limits are hit
     * 
     *  @param forwardLimitEnabled Enable the hardware limit for the forward direction of the motor
     *  @param reverseLimitEnabled Enable the hardware limit for the reverse direction of the motor 
     */
    @Override
    public boolean setZeroOnHardwareLimit(boolean forwardLimitEnabled, boolean reverseLimitEnabled) {
        this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = forwardLimitEnabled;
        this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = reverseLimitEnabled;
        this.config.getMotorConfig().HardwareLimitSwitch.ForwardLimitEnable = forwardLimitEnabled;
        this.config.getMotorConfig().HardwareLimitSwitch.ReverseLimitEnable = reverseLimitEnabled;
        return CTREUtil.Configuration.Motors.applyConfiguration(talon, config) == StatusCode.OK;
    }
    
    /**
     * Set the motion magic config for the current motor
     * 
     * @param config The new config we are using on the motor
     */
    @Override
    public void setSmartMotorConfig(MotionMagicConfigs config) {
       this.config.getMotorConfig().MotionMagic = config;
       CTREUtil.Configuration.Motors.applyConfiguration(talon, config);
    }

    /**
     * Set the voltage configuration for the current motor
     * 
     * @param config The voltage configurations we want to use on the motor
     */
    @Override
    public void setVoltageConfig(VoltageConfigs config) {
        this.config.getMotorConfig().Voltage = config;
        CTREUtil.Configuration.Motors.applyConfigurationNonBlocking(talon, config);
    }

    /**
     * Set the new value that the encoder should report its current location as 
     * 
     * @param position The new position in terms of the subsystem units that the encoder should report here (offsets everything als as a result)
     */
    @Override
    public boolean setCurrentEncoderPosition(double position) {
        return talon.setPosition(getUnitsToRotorRotations(position)) == StatusCode.OK;
    }

    // ------- Motor Control -------
    /**
     * Set a standard duty cycle control value to drive the motor at
     * @param dutyCycle Value between -1 and 1 representing full reverse and full forward respectively 
     */
    @Override
    public boolean setOpenLoopDutyCycle(double dutyCycle) {
        return talon.setControl(dutyCycleControl.withOutput(dutyCycle)) == StatusCode.OK;
    }

    /**
     * Set the voltage with which we want to drive the motor at
     * @param volts The voltage to apply to the motor
     */
    @Override
    public boolean setVoltageOutput(double volts) {
        return talon.setControl(voltageControl.withOutput(volts)) == StatusCode.OK;
    }


    /**
     * Drive the motor at a specified velocity (units as in config) using PID and FF
     * 
     * @param velocity The velocity with which we want to drive the motor
     * @param slot Which slot of gains should we use to achieve this
     */
    @Override
    public boolean setPIDVelocitySetpoint(double velocity, int slot) {
        return talon.setControl(
            velocityVoltageControl
                .withVelocity(velocity)
                .withSlot(slot)
        ) == StatusCode.OK;
    }
    
    /**
     * Set a position setpoint in our real world units to drive the motor to
     * @param posSetpoint The setpoint position in the defined units for this motor
     * @param slot Which slot of gains should we use to achieve this
     */
    @Override
    public boolean setPIDPositionSetpoint(double position, int slot) {
        return talon.setControl(
            positionVoltageControl
                .withPosition(
                    clampPosition(position)
                )
                .withSlot(
                    slot
                )
        ) == StatusCode.OK;
    }

    /**
     * Set the motor to follow another motor mimicing the CAN commmands directly as supplied
     * 
     * @param masterDevice The CAN device we are attempting to mimic
     * @param direction The direction we want this motor to drive in relation to the motor we are mimicing
     */
    @Override
    public boolean follow(CANDeviceID masterDevice, FollowDirection direction) {
        this.getCANDevice().setMasterCANDevice(masterDevice);
        return talon.setControl(
            followerControl
                .withMasterID(masterDevice.getDeviceID())
                .withOpposeMasterDirection(direction == FollowDirection.INVERT)
        ) == StatusCode.OK;
    }

    /**
     * Drive the motor at a given current to optimize torque
     * This doesn't direclty apply current to the motor as brushless motors rotational position and velocity impact the amount of instantaneous torque
     * Instead it useses FOC (Field oriented control) to achieve consistant torque at all times
     * 
     * @param current Current in amps to apply purely to the troque rotation
     */
    @Override
    public boolean setTorqueCurrent(double current) {
        return talon.setControl(
            torqueCurrentFOCControl.withOutput(current)
        ) == StatusCode.OK;
    }

    /**
     * Drive the motor to the motion magic position setpoint in whatever units the subsystem configuration uses
     * 
     * @param position Set the position that we want to drive to using MotionMagic, units are in context of the subsystem config units
     * @param slot Which motion profiling gains slot should we use to drive this control
     * @return True on OK request response False for all else
     */
    @Override
    public boolean setSmartPositionSetpoint(double position, int slot) {
        MotionMagicVoltage mmVoltage = motionMagicPositionControl
                                        .withPosition(
                                            clampPosition(position)
                                        )
                                        .withSlot(
                                            slot
                                        );
             
        return talon.setControl(mmVoltage) == StatusCode.OK;
    }

   /**
     * Drive the motor to the motion magic position setpoint in whatever units the subsystem configuration
     * This method allows for realtime updates to velocity, acceleration, etc. While the profile is already runing
     * 
     * @param position Set the position that we want to drive to using MotionMagic, units are in context of the subsystem config units
     * @param velocity The desired velocity in rotations per second we are trying to hit
     * @param acceleration The desired acceleration we are trying to hit in (rotations per second)^2
     * @param jerk The desired jerk (rate of accelration) we are trying to hit in (rotations per second)^3
     * @param slot Which motion profiling gains slot should we use to drive this control
     * @return True on OK request response False for all else
     */
    @Override
    public boolean setDynamicSmartPositionSetpoint(
        double position, 
        double velocity, 
        double acceleration,
        double jerk, 
        double feedforward,
        int slot
    ) {
        return talon.setControl(
            dynamicMotionMagicPositionControl
                .withPosition(clampPosition(position))
                .withVelocity(velocity)
                .withAcceleration(acceleration)
                .withJerk(jerk)
                .withFeedForward(feedforward)
                .withSlot(slot)
        ) == StatusCode.OK;
    }
    

    /**
     * Attempt to drive the motor velocity (in units of config) at a set velocity using MotionMagic
     * 
     * @param velocity The velocity with which we want the motor to run
     * @param slot The slot of the motor controller gains that we wish to use to achieve our desired setpoint
     */ 
    @Override
    public boolean setSmartVelocitySetpoint(double velocity, int slot) {
        return talon.setControl(
            motionMagicVelocityControl
                .withVelocity(
                    getUnitsToRotorRotations(velocity)
                )
                .withSlot(slot)
        ) == StatusCode.OK;
    }

}
