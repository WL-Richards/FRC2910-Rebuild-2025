// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.google.flatbuffers.Constants;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.lib.config.odometry.OdometryStandardDevs;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.subsystems.drive.DrivetrainIO;
import com.team6443.lib.subsystems.drive.DrivetrainInputs;
import com.team6443.lib.subsystems.simulation.drive.MapleSimSwerveDrivetrain;
import com.team6443.lib.subsystems.simulation.visualizations.DrivetrainVisualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {

  // Reference to the drivetrain and the inputs to the drivetrain
  protected DrivetrainIO drivetrain;
  protected DrivetrainInputs inputs = new DrivetrainInputs();

  // Configuration of this given drivetrain
  protected final DrivetrainConfiguration configuration;

  // Drive train visualizer
  protected final DrivetrainVisualization visualization;

  // What to prepend to logs from this subsystem
  private final String logPrefix;
  
  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem(
    DrivetrainConfiguration configuration,
    DrivetrainIO drivetrain
  ) {
    this.logPrefix = "Subsystems/" + configuration.kConfigurationName;

    this.configuration = configuration;
    this.drivetrain = drivetrain;
    this.drivetrain.setLoggingPrefix(logPrefix);
    this.visualization = new DrivetrainVisualization(configuration.kMaxDriveSpeed, this.logPrefix);
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    drivetrain.updateInputs(inputs);

    // Log the state of the drive train
    visualization.updateViz(inputs);
    Logger.processInputs("RealOutputs/" + logPrefix + "/Inputs", inputs);
    drivetrain.logModules(inputs, this.logPrefix);


    // Update standard deviations based on enable state
    if(DriverStation.isDisabled()){
      configureStandardDevsForDisabled();
    }
    else{
      configureStandardDevsForEnabled();
    }

    // Log Drive train subsystem latency
    Logger.recordOutput(
            this.logPrefix + "/LatencyPeriodicMS", 
            (Timer.getFPGATimestamp() - timestamp)*1000
    );

    // Log the drive train subsystems current command
    Logger.recordOutput(
            this.logPrefix + "/CurrentCommand",
            (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName()
    );
  }

  public DrivetrainSubsystem withStartingPose(Pose2d pose){
    resetOdometry(pose);
    return this;
  }

  // ---- Odometry updates ----
  public void resetOdometry(Pose2d pose) {
    drivetrain.resetOdometry(pose);
  }

  // --- Drive train commanding ----
  public Command applyRequest(Supplier<SwerveRequest> request){
    return drivetrain.continuousRequestCommand(request, this).withName("SwerveDriveRequest");
  }

  public void setControl(SwerveRequest request) {
    drivetrain.setControl(request);
  }

  // ---- Input Deadbanding ----
  protected ChassisSpeeds applyDeadbands(ChassisSpeeds input){
    // Check translational speed
    if (Math.hypot(input.vxMetersPerSecond, input.vyMetersPerSecond) < configuration.kChassisTranslationSpeedThreshold) {
      input.vxMetersPerSecond = input.vyMetersPerSecond = 0.0;
    }

    // Check rotational speed
    if (Math.abs(input.omegaRadiansPerSecond) < configuration.kChassisRotationalSpeedThreshold) {
        input.omegaRadiansPerSecond = 0.0;
    }
    return input;
  }

  // ---- Odometry standard deviation adjustment ----
  protected void setStateStdDevs(OdometryStandardDevs stdDevs){
    drivetrain.setOdometryStdDevs(stdDevs.xStdDev, stdDevs.yStdDev, stdDevs.rotStdDev);
  }
  public void configureStandardDevsForDisabled() {
    setStateStdDevs(this.configuration.kDisabledOdometryStandardDevs);
  }
  public void configureStandardDevsForEnabled() {
      setStateStdDevs(this.configuration.kEnabledOdometryStandardDevs);
  }

  // Attempt to get the sim drive train
  public MapleSimSwerveDrivetrain getSimDrivetrain(){
    if (drivetrain instanceof DrivetrainIOSim){
      return ((DrivetrainIOSim) drivetrain).getMapleSimDrive();
    }

    return null;
  }
}
