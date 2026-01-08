// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.core.motors.MotorInputs;
import com.team6443.lib.core.motors.interfaces.MotorIO;
import com.team6443.lib.subsystems.ServoMotorFollowerSubsystem;
import com.team6443.lib.subsystems.elevator.ElevatorIO;

import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorSubsystem extends ServoMotorFollowerSubsystem<
                                        MotorInputs, 
                                        MotorIO, 
                                        ServoMotorFollowerConfiguration<TalonFXConfiguration>
                                      >
{
  private final ElevatorIO elevator;

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem(
    ServoMotorFollowerConfiguration<TalonFXConfiguration> config,
    ElevatorIO elevator
  ) {

    // Create instance of the servo motor follower subsystem
    super(
      new MotorInputs(), 
      elevator.getLeadMotor(), 
      generateDefaultFollowerInputs(elevator.getFollowerMotors()), 
      elevator.getFollowerMotors(),
      config
    );

    this.elevator = elevator;

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

  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    elevator.updateLog(standardPrefix, inputPrefix);
  }

}
