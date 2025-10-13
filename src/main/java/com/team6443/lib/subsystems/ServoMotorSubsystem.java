// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.team6443.lib.config.motors.ServoMotorConfiguration;
import com.team6443.lib.motors.MotorInputs;
import com.team6443.lib.motors.interfaces.MotorIO;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

/**
 * Subsystem used to drive any MotorIO motor with commands
 */
public class ServoMotorSubsystem<
      I extends MotorInputs, 
      M extends MotorIO, 
      C extends ServoMotorConfiguration
    > 
extends SubsystemBase {

  // The motor IO object itself that we are commanding
  protected M motor;

  // The MotorInputs object that we are using to keep the state of our motor
  protected I motorInputs;

  // What servo motor configuration is being used
  protected C motorConfiguration;

  // The current position in servo motor configuration units of where this motor should be 
  protected double currentPositionSetpoint = 0;

  // Prefix that this motor should ues for logs 
  protected final String logPrefix;

  /**
   * Create new servo motor subsystem with the desired motor and motor config
   * @param motorInputs The motor inputs object that this servo motor is using to track the state
   * @param motor The MotorIO type that this servo motor subsystem is driving
   * @param motorConfiguration The ServoMotorConfiguration that is in use with this system
   */
  public ServoMotorSubsystem(
    I motorInputs, 
    M motor, 
    C motorConfiguration
  ){
    super(motorConfiguration.ConfigurationName);
    this.motorConfiguration = motorConfiguration;
    this.motorInputs = motorInputs;
    this.motor = motor;
    this.logPrefix = motorConfiguration.ConfigurationName + "/" + motor.getName();
  }

  @Override
  public void periodic() {
    motor.updateInputs(motorInputs);
    updateLogs();
  }

  /**
   * Update the logs for this subsystem
   */
  private void updateLogs(){
    // Log the inputs for this motor
    Logger.processInputs(
      logPrefix + "/Inputs", 
      motorInputs
    );

    // Record the current command being executed on this motor
    Logger.recordOutput(
      logPrefix + "/CurrentCommand", 
      (getCurrentCommand() == null) ? "NONE" : getCurrentCommand().getName());
  }
  
  // ---------  Motor Encoder Functionality ---------

  /**
   * Get the current position of this motor subsystem in the context of the defined units
   * @return Current position of the encoder embedded in this motor
   */
  public double getCurrentPosition(){
    return motorInputs.unitPosition;
  }

  /**
   * Get the current velocity of the motor subsystem in the context of the defined units
   * @return Current velocity of the motor in the context of the subsystem units
   */
  public double getCurrentVelocity() {
    return motorInputs.velocityUnitsPerSecond;
  }

  // ---------  Motor Desired State ---------

  /**
   * Get the current position setpoint that this motor is attempting to reach
   * @return The position setpoint in the subsystem units that we are trying to reach with the motor
   */
  public double getPositionSetpointUnits() {
      return currentPositionSetpoint;
  }

  // ---------  Motor IO wrappers ---------

  // ------  Motor Configuration ------
  
  protected void setSmartMotionConfigImpl(MotionMagicConfigs config){
    motor.setSmartMotorConfig(config);
  }

  protected void setNeutralModeImpl(MotorIO.NeutralMode mode){
    Logger.recordOutput(logPrefix + "/SetNeutralMode", mode.toString());
    motor.setNeutralMode(mode);
  }

  // ------  Duty Cycle Control ------

  protected void setOpenLoopDutyCycleImpl(double dutyCycle){
    Logger.recordOutput(logPrefix + "/SetOpenLoopDutyCycle", dutyCycle);
    motor.setOpenLoopDutyCycle(dutyCycle);
  }

  // ------  Voltage Control ------

  protected void setVoltageImpl(double voltage){
    Logger.recordOutput(logPrefix + "/SetVoltage", voltage);
    motor.setVoltageOutput(voltage);
  }
  
  // ------  Torque Control ------

  protected void setTorqueCurrentImpl(double current){
    Logger.recordOutput(logPrefix + "/SetTorqueCurrent", current);
    motor.setTorqueCurrent(current);
  }

  // ------  PID Velocity Control ------

  protected void setPIDVelocitySetpointImpl(double velocity, int slot){
    Logger.recordOutput(logPrefix + "/SetPIDVelocitySetpoint", velocity);
    motor.setPIDVelocitySetpoint(velocity, slot);
  }

  protected void setPIDVelocitySetpointImpl(double velocity){
    setPIDVelocitySetpointImpl(velocity, 0);
  }
  
  // ------  PID Position Control ------

  protected void setPIDPositionSetpointImpl(double position, int slot){
    currentPositionSetpoint = position;
    Logger.recordOutput(logPrefix + "/SetPIDPositionSetpoint", position);
    motor.setPIDPositionSetpoint(position, slot);
  }

  protected void setPIDPositionSetpointImpl(double position){
    setPIDPositionSetpointImpl(position, 0);
  }
  
  // ------  Motion Profiled Position Control ------

  protected void setSmartPositionSetpointImpl(double position){
    setSmartPositionSetpointImpl(position, 0);
  }

  protected void setSmartPositionSetpointImpl(double position, int slot){
      currentPositionSetpoint = position;
      Logger.recordOutput(logPrefix + "/SetSmartPositionSetpoint/Position", position);
      Logger.recordOutput(logPrefix + "/SetSmartPositionSetpoint/Slot", position);
      motor.setSmartPositionSetpoint(position, slot);
  }

  // ------ Dynamic Motion Profiled Position Control ------
  
  protected void setDynamicSmartPositionSetpointImpl(double position, MotionMagicConfigs config, int slot){
    setDynamicSmartPositionSetpointImpl(
      position, 
      config.MotionMagicCruiseVelocity, 
      config.MotionMagicAcceleration, 
      config.MotionMagicJerk, 
      0.0,
      slot
    );
  }

  protected void setDynamicSmartPositionSetpointImpl(double position, double feedforward, MotionMagicConfigs config, int slot){
    setDynamicSmartPositionSetpointImpl(
      position, 
      config.MotionMagicCruiseVelocity, 
      config.MotionMagicAcceleration, 
      config.MotionMagicJerk, 
      feedforward,
      slot
    );
  }

  protected void setDynamicSmartPositionSetpointImpl(
    double position, 
    double velocity, 
    double acceleration,
    double jerk, 
    double feedforward,
    int slot
  ){
    currentPositionSetpoint = position;
    Logger.recordOutput(logPrefix + "/SetSmartPositionSetpoint/Position", position);
    Logger.recordOutput(logPrefix + "/SetSmartPositionSetpoint/Slot", position);
    motor.setDynamicSmartPositionSetpoint(position, velocity, acceleration, jerk, feedforward, slot);
  }

  // ------ Dynamic Motion Profiled Velocity Control ------

  protected void setSmartVelocitySetpointImpl(double velocity, int slot){
    Logger.recordOutput(logPrefix + "/SetSmartVelocitySetpoint/Velocity", velocity);
    Logger.recordOutput(logPrefix + "/SetSmartVelocitySetpoint/Slot", slot);
    motor.setSmartVelocitySetpoint(velocity, slot);
  }

  protected void setSmartVelocitySetpointImpl(double velocity){
    setSmartVelocitySetpointImpl(velocity, 0);
  }
 
  // --------- Command Implementation ---------

  // ------  Motor Configuration ------

  /**
   * Set the smart motion config for the given motor subsystem
   * @param config The configuration we want to use for this given motor
   * @return reference to the command base object so we can chain
   */
  public Command smartMotionConfigCommand(MotionMagicConfigs config){
    /* 
     We don't want to want to require the current subsystem here because we don't *really* 
     need the subsystem exclusively as we are just changing the config
    */
    return new InstantCommand(
      () -> setSmartMotionConfigImpl(config)
    )
    .withName(getName() + "_SetSmartMotionConfig");
  }

  // ------  Control Modifiers ------

  public Command withoutSoftwareLimitsTemporailyCommand(){
    // Temp command to store the limits
    var prevLimits =
      new Object() {
          boolean forwardLimitsEnabled = false;
          boolean reverseLimitsEnabled = false;

          void fromPair(Pair<Boolean, Boolean> limits){
            this.forwardLimitsEnabled = limits.getFirst();
            this.reverseLimitsEnabled = limits.getSecond();
          }
      };

    // This doesn't have any requirements because it shouldn't require the subsystem
    return Commands.startEnd(
      // When the command first starts we want to save the state of the software limits so we can 
      () -> {
        prevLimits.fromPair(motor.getEnableSoftwareLimits());
        motor.setEnableSoftwareLimits(
          false, 
          false
        );
      },     
      () -> {
        motor.setEnableSoftwareLimits(
          prevLimits.forwardLimitsEnabled, 
          prevLimits.reverseLimitsEnabled
        );
      }
    ).withName(getName() + "_WithoutSoftwareLimitsTemp");
  }

  /**
   * Run the command passed in with the software limits disabled renabling them once the command is complete
   * @param commandToRun The command we wish to run with the software limits enabled
   * @return The command that is being run
   */
  public Command runWithoutSoftwareLimitsCommnand(Command commandToRun){
    return new ParallelDeadlineGroup(
      commandToRun,
      withoutSoftwareLimitsTemporailyCommand()
    ).withName(getName() + "Running_" + commandToRun.getName() + "_WithoutSoftwareLimitsTemp");
    
  }
  // ------ Duty Cycle Commands ------

  /**
   * Command the motor to run at a given duty cycle
   * @param dutyCycle Double supplier supplying the duty cycle (-1 to 1)
   * @return Reference to the command being run
   */
  public Command dutyCycleCommand(DoubleSupplier dutyCycle){
    return runEnd(
      () -> { setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble()); },
      () -> { setOpenLoopDutyCycleImpl(0.0); }
    )
    .withName(getName() + "_DutyCycleControl");
  } 

  /**
   * Command the motor to run at a given duty cycle
   * @param dutyCycle Double supplier supplying the duty cycle (-1 to 1)
   * @return Reference to the command being run
   */
  public Command dutyCycleCommandNoEnd(DoubleSupplier dutyCycle){
    return runEnd(
      () -> { setOpenLoopDutyCycleImpl(dutyCycle.getAsDouble()); },
      () -> {}
    )
    .withName(getName() + "_DutyCycleControl");
  } 

  // ------ Voltage Commands ------

  /**
   * Drive the motor at a specified voltage until canceled 
   * @param voltage Voltage to drive the motor at
   * @return The command that is being run
   */
  public Command voltageCommand(DoubleSupplier voltage){
    return runEnd(
      () -> { setVoltageImpl(voltage.getAsDouble()); },
      () -> { setVoltageImpl(0.0); }
    )
    .withName(getName() + "_VoltageControl");
  }
  
  // --- Torque Control Commands ---

  public Command torqueCurrentCommand(DoubleSupplier current){
    return runEnd(
      () -> { setTorqueCurrentImpl(current.getAsDouble()); }, 
      () -> { setTorqueCurrentImpl(0.0); }
    ).withName(getName() + "_TorqueCurrent");
  }

  // ------ Velocity Control Commands ------

  public Command velocitySetpointCommand(DoubleSupplier velocity){
    return runEnd(
      () -> { setPIDVelocitySetpointImpl(velocity.getAsDouble()); }, 
      () -> {}
    )
    .withName(getName() + "_PIDVelocityControl");
  }
  
  public Command smartVelocitySetpointCommand(DoubleSupplier velocity, int slot){
    return runEnd(
      () -> { setSmartVelocitySetpointImpl(velocity.getAsDouble(), slot); }, 
      () -> {}
    )
    .withName(getName() + "_SmartVelocityControl");
  }

  public Command smartVelocitySetpointCommand(DoubleSupplier velocity){
    return smartVelocitySetpointCommand(
      velocity, 
      0
    );
  }

  // ------ Coast / Brake Commands ------

  /**
   * Set the neutral mode of the motor to be coast mode
   * @return Reference to the command that was run
   */
  public Command setCoastCommand(){
    return new InstantCommand( 
      () -> {setNeutralModeImpl(MotorIO.NeutralMode.COAST); }
    )
    .withName(getName() + "_SetCoast");
  }

  /**
   * Set the neutral mode of the motor to be brake mode
   * @return Reference to the command that was run
   */
  public Command setBrakeCommand(){
    return new InstantCommand( 
      () -> {setNeutralModeImpl(MotorIO.NeutralMode.BRAKE); }
    )
    .withName(getName() + "_SetBrake");
  }

  // ------ Position Control Commands ------

  // --- Basic Control Commands ---
  public Command positionSetpointCommand(DoubleSupplier position, int slot){
    return runEnd(
      () -> { setPIDPositionSetpointImpl(position.getAsDouble(), slot); }, 
      () -> {}
    )
    .withName(getName() + "_PIDPositionControl");
  }

  public Command positionSetpointUntilOnTargetCommand(DoubleSupplier position, DoubleSupplier acceptableError, int slot){
    return positionSetpointCommand(position, slot)
      .until(
        () -> MathUtil.isNear(position.getAsDouble(), motorInputs.unitPosition, acceptableError.getAsDouble()))
      .withName(getName() + "_PIDPositionControlUntilOnTarget");
  }

  public Command positionSetpointUntilOnTargetCommand(DoubleSupplier position, DoubleSupplier acceptableError){
    return positionSetpointCommand(position, 0)
      .until(
        () -> MathUtil.isNear(position.getAsDouble(), motorInputs.unitPosition, acceptableError.getAsDouble()))
      .withName(getName() + "_PIDPositionControlUntilOnTarget");
  }

  // --- Smart Control Commands ---

  public Command smartPositionSetpointCommand(DoubleSupplier position, int slot){
    return runEnd(
      () -> { setSmartPositionSetpointImpl(position.getAsDouble(), slot); }, 
      () -> {}
    )
    .withName(getName() + "_SmartPositionControl");
  }

  public Command smartPositionSetpointCommand(DoubleSupplier position){
    return smartPositionSetpointCommand(position, 0);
  }

  public Command smartPositionSetpointUntilOnTargetCommand(DoubleSupplier position, DoubleSupplier acceptableError, int slot){
    return smartPositionSetpointCommand(position, slot)
      .until(
        () -> MathUtil.isNear(position.getAsDouble(), motorInputs.unitPosition, acceptableError.getAsDouble())
      )
      .withName(getName() + "_SmartPositionControlUntilOnTarget");
  } 

  public Command smartPositionSetpointUntilOnTargetCommand(DoubleSupplier position, DoubleSupplier acceptableError){
    return smartPositionSetpointUntilOnTargetCommand(position, acceptableError, 0);
  } 

  // --- Dynamic Smart Control Commands ---
  
  public Command dynamicSmartPositionSetpointCommand(DoubleSupplier position, Supplier<MotionMagicConfigs> config, int slot){
    return runEnd(
      () -> { setDynamicSmartPositionSetpointImpl(position.getAsDouble(), config.get(), slot); },
      () -> {}
      ).withName(getName() + "_DynamicSmartPositionControl");
  }

  public Command dynamicSmartPositionSetpointCommand(DoubleSupplier position, Supplier<MotionMagicConfigs> config){
    return dynamicSmartPositionSetpointCommand(position, config, 0);
  }

  public Command dynamicSmartPositionSetpointUntilOnTargetCommand(
    DoubleSupplier position, 
    DoubleSupplier acceptableError, 
    Supplier<MotionMagicConfigs> config, 
    int slot
  ){
    return dynamicSmartPositionSetpointCommand(position, config, slot)
      .until(
        () -> MathUtil.isNear(position.getAsDouble(), motorInputs.unitPosition, acceptableError.getAsDouble())
      )
      .withName(getName() + "_DynamicSmartPositionControlUntilOnTarget");
  }

  public Command dynamicSmartPositionSetpointUntilOnTargetCommand(
    DoubleSupplier position, 
    DoubleSupplier acceptableError, 
    Supplier<MotionMagicConfigs> config
  ){
    return dynamicSmartPositionSetpointUntilOnTargetCommand(position, acceptableError, config, 0);
  }
}
