// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.motors.interfaces;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.motors.MotorInputs;

/** 
 * Base motor interface gives us a common method for interfacing with motors
 */
public interface MotorIO {

    /**
     * Basic neutral mode enum, to make it API agnostic
     */
    public enum NeutralMode {
        COAST,
        BRAKE
    }


    /**
     * The direction we are attempting to follow the master device in relation to the master's direction
     */
    public enum FollowDirection {
        INVERT,
        SAME
    }

    // ------ Basic Motor Config  ------
    /**
     * Enable or disabled forward and backwards software limits
     * @param forwardLimitEnabled Should we enable (true) or disable (false) the forward direction software limits
     * @param reversLimitEnabled Should we enable (true) or disable (false) the reverse direction software limits
     * @return true on success false on failure
     */
    public boolean setEnableSoftwareLimits(boolean forwardLimitEnabled, boolean reversLimitEnabled);

    /**
     * Enable or disabled forward and backwards hardware limits
     * @param forwardLimitEnabled Should we enable (true) or disable (false) the forward direction hardware limits from connected limit switches
     * @param reversLimitEnabled Should we enable (true) or disable (false) the reverse direction hardware limits from connected limit switches
     * @return true on success false on failure
     */
    public boolean setEnableHardwareLimits(boolean forwardLimitEnabled, boolean reversLimitEnabled);

    /**
     * Enable or disabled the functionality to zero the motors position (Where it thinks it is, doesn't actually drive the motor) when a hardware limit switch is hit
     * NOTE: this function doesn't enable or disable the hardware limit switches themselves just sets the auto position field
     * @param forwardLimitEnabled Should we enable (true) or disable (false) the forward direction hardware limits from connected limit switches
     * @param reversLimitEnabled Should we enable (true) or disable (false) the reverse direction hardware limits from connected limit switches
     * @return true on success false on failure
     */
    public boolean setZeroOnHardwareLimit(boolean forwardLimitEnabled, boolean reversLimitEnabled);

    /**
     * Set this motor to do exactly what a different motor with some other CAN id is doing at exactly the same time
     * @param masterDevice CAN device that we are following
     * @param direction The direction we are following the device at (Inverted or same)
     * @return true on success false on failure
     */
    public boolean follow(CANDeviceID masterDevice, FollowDirection direction);

    /**
     * Overload to configure motor with motion magic configuration
     * @param config The motion magic config to apply for this motor
     */
    void setSmartMotorConfig(MotionMagicConfigs config);
    

    /**
     * Configure the voltage output of this motor
     * @param config The voltage configuration to use
     */
    void setVoltageConfig(VoltageConfigs config);

    // ------ Encoder Basics ------
    /**
     * Set the encoders new relative position
     * Example:
     *      - Reported Encoder Value: 21.0
     *      - setCurrentEncoderPosition(0.0)
     *      - Reported Encoder Value: 0.0
     * In the above example whatever position the encoder was reporting as 21 is now what it thinks is zero
     * This is not preserved between robot restarts as relative encoder values are *relative*
     * 
     * @param position The new position the encoder should be reporting 
     * @return true on success false on failure
     */
    public boolean setCurrentEncoderPosition(double position);

    /**
     * Sets the current encoder location to be the new relative zero
     * @return true on success false on failure
     */
    public default boolean zeroEncoderPosition(){
        return setCurrentEncoderPosition(0.0);
    }

    // ------ Motor outputs ------
    /**
     * Update the MotorInputs object passed in with the current inputs to the motor
     * @param inputs The inputs that are currently applied to the motor
     * @return true on success false on failure
     */
    public boolean updateInputs(MotorInputs inputs);

    // ------ Basic Motor Control  ------
    /**
     * Apply a given number of volts to this motor
     * @param volts The amount of volts desired on this motor
     * @return true on success false on failure
     */
    public boolean setVoltageOutput(double volts);

    /**
     * Define an amount of current we want to apply direclty to torque
     * Might be CTRE only (unsure)
     * @param current Amount of current in Amps to be driven into torque
     * @return true on success false on failure
     */
    public boolean setTorqueCurrent(double current);

    /**
     * Apply a duty cycle to the motor with no feedback
     * @param dutyCycle Value between -1 and 1 to represent full reverse and full forward respectively
     * @return true on success false on failure
     */
    public boolean setOpenLoopDutyCycle(double dutyCycle);

    /**
     * Set the state for how the motor behaves when no input is applied
     * @param mode The mode that this motor should be in when no input is applied
     * @return true on success false on failure
     */
    public boolean setNeutralMode(NeutralMode mode);

    /**
     * Drive the motor to some position using standard PID / FF
     * @param position The position with which the motor should be driven to (these units are relative to whatever config is used on this motor)
     * @return true on success false on failure
     */
    public boolean setPIDPositionSetpoint(double position, int slot);

     /**
     * Drive the motor to at some velocity using standard PID / FF
     * @param posSetpoint The velocity with which the motor should be driven to (these units are relative to whatever config is used on this motor)
     * @return true on success false on failure
     */
    public boolean setPIDVelocitySetpoint(double velocity, int slot);

    // ------ Smart Motor Output: Position Control ------

    // --- Static Profiling Control ---
    /**
     * Define a position set point for the motor to drive to with feedback (the smart part)
     * Examples of smart position set points would include motion magic (CTRE) or smart motion (REV)
     * @param position The position with which the motor should be driven to (these units are relative to whatever config is used on this motor)
     * @param slot The slot in the gains bank to use to drive the position
     * @return true on success false on failure
     */
    public boolean setSmartPositionSetpoint(double position, int slot);


    /**
     * Define a position set point for the motor to drive to with feedback (the smart part)
     * Examples of smart position set points would include motion magic (CTRE) or smart motion (REV)
     * 
     * Assume slot 0 of the gains bank
     * @param posSetpoint The position with which the motor should be driven to (these units are relative to whatever config is used on this motor)
     * @return true on success false on failure
     */
    public default boolean setSmartPositionSetpoint(double position){
        return setSmartPositionSetpoint(position, 0);
    };

    // --- Dynamic Profiling Control ---
    /**
     * Define a position set point for the motor to drive to with feedback (the smart part) and allows for realtime updating of profiling gains 
     * This feature may only exist on CTRE Talon devices (unsure)
     * @param position The position with which the motor should be driven to (these units are relative to whatever config is used on this motor)
     * @param velocity Velocity at which we wish to drive the motor at
     * @param acceleration Acceleration at which we wish to drive the motor
     * @param jerk The rate at which we can accelerate
     * @param feedforward The feedforward gains for the motor in volts
     * @param slot Gain slot to use on the device to drive the motion profile
     * @return true on success false on failure
     */
    public boolean setDynamicSmartPositionSetpoint(
        double position, 
        double velocity, 
        double acceleration, 
        double jerk,
        double feedforward,
        int slot
    );

    /**
     * Define a position set point for the motor to drive to with feedback (the smart part) and allows for realtime updating of profiling gains 
     * This feature may only exist on CTRE Talon devices (unsure)
     * @param posSetpoint The position with which the motor should be driven to (these units are relative to whatever config is used on this motor)
     * @param velocity Velocity at which we wish to drive the motor at
     * @param acceleration Acceleration at which we wish to drive the motor
     * @param jerk The rate at which we can accelerate
     * @param feedforward Feedforward to apply in volts to the motor
     * @return true on success false on failure
     */
    public default boolean setDynamicSmartPositionSetpoint(
        double position, 
        double velocity, 
        double acceleration, 
        double jerk,
        double feedforward
    )
    {
        return setDynamicSmartPositionSetpoint(position, velocity, acceleration, jerk, feedforward, 0);
    };

    // ------ Smart Motor Output: Smart Velocity Control ------

    // --- Static Profiling Control ---
    /**
     * Define a velocity set point for the motor to drive at with feedback (the smart part)
     * Examples of smart position set points would include motion magic (CTRE) or smart motion (REV)
     * @param unitsPerSecond The velocity setpoint with which the motor should be driven
     * @param slot The slot in the gains bank to use to drive the motor to the desired velocity and maintain it
     * @return true on success false on failure
     */
    public boolean setSmartVelocitySetpoint(double unitsPerSecond, int slot);

    /**
     * Define a velocity set point for the motor to drive at with feedback (the smart part)
     * Examples of smart position set points would include motion magic (CTRE) or smart motion (REV)
     * @param unitsPerSecond The velocity setpoint with which the motor should be driven
     * @return true on success false on failure
     */
    public default boolean setSmartVelocitySetpoint(double unitsPerSecond){
        return setSmartVelocitySetpoint(unitsPerSecond, 0);
    }; 

    

}
