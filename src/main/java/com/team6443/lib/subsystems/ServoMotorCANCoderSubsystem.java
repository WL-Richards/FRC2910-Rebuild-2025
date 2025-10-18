// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems;

import org.littletonrobotics.junction.Logger;

import com.team6443.lib.config.motors.ServoMotorCANCoderConfiguration;
import com.team6443.lib.encoders.CANCoderInputs;
import com.team6443.lib.encoders.interfaces.CANCoderIO;
import com.team6443.lib.motors.MotorInputs;
import com.team6443.lib.motors.interfaces.MotorIO;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;

/**
 * This class is intended for the use case when a CAN coder is desired to be used instead of the motors internal encoder, and simply uses the CAN coder to update what the "motor" encoder reports to match
 */
public class ServoMotorCANCoderSubsystem<
              MI extends MotorInputs,                         // Motor inputs class to be used, eg. MotorInputs
              M extends MotorIO,                              // Motor interface to be used within the subsystem, eg. TalonFXIO
              EI extends CANCoderInputs,                      // CANCoder inputs class to be used, eg. CANCoderInputs
              E extends CANCoderIO,                           // Interface of the CAN coder object that we are using
              C extends ServoMotorCANCoderConfiguration<?>    // What servo motor configuration to be using, eg. S
            >
 extends ServoMotorSubsystem<MI, M, C> {
  
  // Inputs to be used for this CANcoder
  protected EI canCoderInputs;

  // The instance of CANCoder IO itself
  protected E canCoder;

  // Has the "motors" position been offset yet to use the CANcoders position instead of the internal motor position so that we always know where the motor is on initialization
  protected boolean hasSetOffset = false;

  // Prefix that this motor should ues for logs 
  private String logPrefix;
  
  /** Creates a new ServoMotorSubsystemCanCoder. */
  public ServoMotorCANCoderSubsystem(
    MI motorInputs, 
    M motor, 
    EI canCoderInputs,
    E canCoder,
    C motorConfiguration
  ) {
    super(motorInputs, motor, motorConfiguration);
    this.canCoderInputs = canCoderInputs;
    this.canCoder = canCoder;
    this.logPrefix = "RobotState/Subsystems/" + motorConfiguration.ConfigurationName + "/" + canCoder.getName();
  }

  @Override
  public void periodic() {
    super.periodic();

    // Update the state of the CAN coder
    canCoder.updateInputs(canCoderInputs);
    Logger.processInputs("RealOutputs/" + logPrefix + "/Inputs",  canCoderInputs);

    // If this encoder is not fused, has a valid location and the offset hasn't already been set we want to update the motors position to the same value
    if(!this.config.isFusedCANCoder && !this.hasSetOffset && !Double.isNaN(canCoderInputs.absolutePositionRotations)){
      updateOffsetImpl();
      this.hasSetOffset = true;
    }
  }

  /**
   * Implementation to update what the motor thinks its encoder position is to match what the CAN coder is telling it (since absolute it gives us a really solid zero)
   */
  protected void updateOffsetImpl(){
      motor.setCurrentEncoderPosition(canCoderInputs.absolutePositionRotations * this.config.CANCoderRotationToUnitRatio);
  }

  /**
   * Command to run to update the offset applied to the motor's encoder, effectively a re-zero
   * @return The command that was run for chaining
   */
  public Command updateCANCoderOffsetCommand(){
    return new InstantCommand(() -> updateOffsetImpl());
  }


}
