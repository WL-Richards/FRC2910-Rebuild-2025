// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.motors.MotorInputs;
import com.team6443.lib.motors.interfaces.MotorIO;
import com.team6443.lib.subsystems.ServoMotorFollowerSubsystem;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends ServoMotorFollowerSubsystem<
                                        MotorInputs, 
                                        MotorIO, 
                                        ServoMotorFollowerConfiguration<TalonFXConfiguration>
                                      >
{

  private String logPrefix;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(
    ServoMotorFollowerConfiguration<TalonFXConfiguration> config,
    MotorIO leadMotor,
    MotorIO[] followerMotors
  ) {

    // Create instance of the servo motor follower subsystem
    super(
      new MotorInputs(), 
      leadMotor, 
      generateDefaultFollowerInputs(followerMotors), 
      followerMotors,
      config
    );

    this.logPrefix = "Subsystems/" + config.kConfigurationName;

    // Zero the encoders, set the default command to effectively "hold position"
    zeroEncoderPosition();
    setDefaultCommand(holdPositionCommand());
  }

  @Override
  public void periodic() {
    super.periodic();
  }

  public Command holdPositionCommand() {
    return smartPositionSetpointCommand(this::getPositionSetpointUnits)
      .withName("HoldPosition")
      .ignoringDisable(true);
  }

}
