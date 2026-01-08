// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.drive.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

/* 
 * Drive heading command to power the swerve drive train control
*/
public class DriveWithHeadingCommand extends Command {
  private final DrivetrainSubsystem drivetrainSubsystem;

  // Inputs to the drive train
  private final DoubleSupplier throttleSupplier;
  private final DoubleSupplier strafeSupplier;
  private final DoubleSupplier turnSupplier;

  private final double kMaxDriveSpeed;
  private final double kMaxAngularRate;

  private final double kJoystickSteerDeadband;
  private final double kJoystickDriveDeadband;

  // The desired rotation of the robot if one is currently present
  private Optional<Rotation2d> desiredHeading = Optional.empty();

  // -1 is unset, the last time input was given to the turn supplier
  private double joystickLastTurnTime = -1;

  // --- Control Modes ---
  private final SwerveRequest.FieldCentric driveNoHeading;

  private final SwerveRequest.FieldCentricFacingAngle driveWithHeading;

  public DriveWithHeadingCommand(
    DrivetrainSubsystem subsystem, 
    DrivetrainConfiguration driveTrainConfiguration,
    DoubleSupplier throttle,
    DoubleSupplier strafe,
    DoubleSupplier turn
  ) {
     this.drivetrainSubsystem = subsystem;
     addRequirements(this.drivetrainSubsystem);

     this.throttleSupplier = throttle;
     this.strafeSupplier = strafe;
     this.turnSupplier = turn;

     // Fetch once, every loop is slow
     this.kMaxDriveSpeed = driveTrainConfiguration.kMaxDriveSpeed;
     this.kMaxAngularRate = driveTrainConfiguration.kMaxAngularRate;
     this.kJoystickSteerDeadband = driveTrainConfiguration.kSteerJoystickDeadband;
     this.kJoystickDriveDeadband = driveTrainConfiguration.kDriveJoystickDeadband;

      // Field centric drive WITHOUT heading lock set
      driveNoHeading =
          new SwerveRequest.FieldCentric()
              .withDeadband(
                driveTrainConfiguration.kChassisTranslationSpeedThreshold * driveTrainConfiguration.kDriveJoystickDeadband
              )
              .withRotationalDeadband(
                driveTrainConfiguration.kChassisRotationalSpeedThreshold * driveTrainConfiguration.kSteerJoystickDeadband
              )
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    
      // Field centric drive WITH heading lock
      driveWithHeading = 
          new SwerveRequest.FieldCentricFacingAngle()
              .withDeadband(
                driveTrainConfiguration.kChassisTranslationSpeedThreshold * driveTrainConfiguration.kDriveJoystickDeadband
              )
              .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.desiredHeading = Optional.empty();     // Clear desired heading on command initialize
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double throttle = this.throttleSupplier.getAsDouble() * this.kMaxDriveSpeed;
    double strafe = this.strafeSupplier.getAsDouble() * this.kMaxDriveSpeed;

    double turn = this.turnSupplier.getAsDouble();

    boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Red;
    double throttleAllianceRelative = isRedAlliance ? -throttle : throttle;
    double strafeAllianceRelative = isRedAlliance ? -strafe : strafe;

    if (Math.abs(turn) > this.kJoystickSteerDeadband){
      joystickLastTurnTime = Timer.getFPGATimestamp();
    }

    // We should only update our rotational input if the user is providing input or was recently providing input and the rotation rate is still greater than 10 degrees per second
    if (Math.abs(turn) > this.kJoystickSteerDeadband) 
    
    // This is used if we are driving wit
    // IF we are attempting to turn the robot
    //       || // // OR our last turn joystick time was within .25 seconds of input (this is just to allow the joystick to settle) AND the bot is still rotating too quickly (more than 10 degress per second)
    //       ((MathUtil.isNear(joystickLastTurnTime, Timer.getFPGATimestamp(), 0.25))  
    //             && Math.abs(                                                                     
    //                 RobotState.get().getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond
    //       ) >
    //       Math.toRadians(10))
    //   )   
    {
      // Command the subsystem to drive with the given rotational rate
      drivetrainSubsystem.setControl(
        driveNoHeading
          .withVelocityX(throttleAllianceRelative)
          .withVelocityY(strafeAllianceRelative)
          .withRotationalRate(
              -turn * this.kMaxAngularRate
          )
      );

      this.desiredHeading = Optional.empty();
    }

    // // ---- If there is not rotational 
    else 

    {

      drivetrainSubsystem.setControl(
        driveNoHeading
          .withVelocityX(Math.abs(throttleAllianceRelative) > this.kJoystickSteerDeadband ? throttleAllianceRelative : 0)
          .withVelocityY(Math.abs(strafeAllianceRelative) > this.kJoystickSteerDeadband ? strafeAllianceRelative : 0 )
          .withRotationalRate(
            0.0
          )
      );

    }




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


}
