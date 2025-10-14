// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.config.motors.ServoMotorCANCoderConfiguration;
import com.team6443.lib.encoders.CANCoderInputs;
import com.team6443.lib.encoders.interfaces.CANCoderIO;
import com.team6443.lib.motors.MotorInputs;
import com.team6443.lib.motors.interfaces.MotorIO;
import com.team6443.lib.subsystems.ServoMotorCANCoderSubsystem;

public class TestSubsystem extends ServoMotorCANCoderSubsystem<
                                      MotorInputs, 
                                      MotorIO, 
                                      CANCoderInputs,
                                      CANCoderIO,
                                      ServoMotorCANCoderConfiguration<TalonFXConfiguration>
                                    > {
  /** Creates a new TestSubsystem. */
  public TestSubsystem(
    final ServoMotorCANCoderConfiguration<TalonFXConfiguration> config,
    final MotorIO motor,
    final CANCoderIO canCoder
  ) {
    super(
      new MotorInputs(), 
      motor, 
      new CANCoderInputs(), 
      canCoder, 
      config
    );
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
