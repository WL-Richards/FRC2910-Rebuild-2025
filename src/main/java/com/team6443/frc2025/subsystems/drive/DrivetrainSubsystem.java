// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.drive;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team6443.frc2025.state.RobotState;
import com.team6443.frc2025.subsystems.drive.io.DrivetrainSimIO;
import com.team6443.lib.config.odometry.OdometryStandardDevs;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.core.logging.Loggable;
import com.team6443.lib.subsystems.AEMSubsystem;
import com.team6443.lib.subsystems.drive.DrivetrainIO;
import com.team6443.lib.subsystems.drive.DrivetrainInputs;
import com.team6443.lib.subsystems.drive.simulation.MapleSimSwerveDrivetrain;
import com.team6443.lib.subsystems.drive.visualizations.SwerveVisualizer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;

public class DrivetrainSubsystem extends AEMSubsystem {

  // Game field reference
  private final Field2d field = new Field2d();

  // Reference to the drivetrain and the inputs to the drivetrain
  protected DrivetrainIO drivetrain;
  protected DrivetrainInputs inputs = new DrivetrainInputs();

  // Loggable interface representation of the choreo pathing, cause we don't really care about the actual object
  protected Loggable ChoreoPatherLoggable = null;

  // Configuration of this given drivetrain
  protected final DrivetrainConfiguration configuration;

  //
  private final SwerveVisualizer swerveViz;
  
  /** Creates a new DriveSubsystem. */
  public DrivetrainSubsystem(
    DrivetrainConfiguration configuration,
    DrivetrainIO drivetrain
  ) {
    super(configuration.kConfigurationName);
    
    // Helper class for creating swerve state mechanism
    swerveViz = new SwerveVisualizer(configuration.kMaxDriveSpeed);

    this.configuration = configuration;
    this.drivetrain = drivetrain;

    RobotState.get().registerDriveTrainVisionEstimateConsumer(
        (estimate) -> drivetrain.addVisionMeasurement(
            estimate.getTimestampSeconds(),
            estimate.getVisionRobotPoseMeters(),
            estimate.getVisionMeasurementStdDevs()
        )
    );
  }

  @Override
  public void periodic() {
    double timestamp = Timer.getFPGATimestamp();
    drivetrain.updateInputs(inputs);

    // Log the state of the drive train
    updateLog();
    updateRobotState();
    drivetrain.logModules(inputs, kLogPrefixStandard);


    // Update standard deviations based on enable state
    if(DriverStation.isDisabled()){
      configureStandardDevsForDisabled();
    }
    else{
      configureStandardDevsForEnabled();
    }

    // Log Drive train subsystem latency
    Logger.recordOutput(
            kLogPrefixStandard + "/LatencyPeriodicMS", 
            (Timer.getFPGATimestamp() - timestamp)*1000
    );

    // Log the drive train subsystems current command
    Logger.recordOutput(
            kLogPrefixStandard + "/CurrentCommand",
            (getCurrentCommand() == null) ? "Default" : getCurrentCommand().getName()
    );
  }

  private void updateRobotState(){
    // ------ Update Robot State ------
    // Extract values from inputs
    double timestamp = inputs.Timestamp; // Use the timestamp logged with the data!
        
    // Convert units (Logic previously in HardwareIO)
    double rollRadsPerS = Units.degreesToRadians(inputs.rollAngularVelocity);
    double pitchRadsPerS = Units.degreesToRadians(inputs.pitchAngularVelocity);
    double yawRadsPerS = Units.degreesToRadians(inputs.Speeds.omegaRadiansPerSecond); // Or derive from gyro diff
    double pitchRads = Units.degreesToRadians(inputs.pitch);
    double rollRads = Units.degreesToRadians(inputs.roll);

    // Calculate Chassis Speeds (Logic previously in HardwareIO)
    // We can trust inputs.Speeds (Field Relative) and inputs.Pose which come from the SwerveDriveState
    ChassisSpeeds actualRobotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
        inputs.Speeds, 
        inputs.Pose.getRotation()
    );
    
    // We can construct the "Gyro Fused" speed if we want to trust gyro rate over odometry rate
    ChassisSpeeds gyroFusedFieldRelative = new ChassisSpeeds(
        inputs.Speeds.vxMetersPerSecond,
        inputs.Speeds.vyMetersPerSecond,
        yawRadsPerS
    );

    // Update RobotState
    RobotState.get().addChassisMotionMeasurements(
        timestamp, 
        rollRadsPerS, 
        pitchRadsPerS, 
        yawRadsPerS, 
        pitchRads, 
        rollRads, 
        inputs.accelX, 
        inputs.accelY, 
        actualRobotRelative, 
        inputs.Speeds, // Actual Field Relative
        drivetrain.getSwerveKinematics().toChassisSpeeds(inputs.ModuleTargets), // Desired Robot Relative
        new ChassisSpeeds(), // Desired Field Relative (Calculable if needed)
        gyroFusedFieldRelative
    );
  }

  public DrivetrainSubsystem withStartingPose(Pose2d pose){
    resetOdometry(pose);
    return this;
  }


  // ---- Logging ----
  @Override
  public void updateLog(String standardPrefix, String inputPrefix) {
    Logger.processInputs(inputPrefix + "/Inputs", inputs);

    Logger.recordOutput(standardPrefix + "/Odometry/Speed", new Translation2d(inputs.Speeds.vxMetersPerSecond, inputs.Speeds.vyMetersPerSecond).getNorm());
    Logger.recordOutput(standardPrefix + "/Odometry/VelocityX", inputs.Speeds.vxMetersPerSecond);
    Logger.recordOutput(standardPrefix + "/Odometry/VelocityY", inputs.Speeds.vyMetersPerSecond);
    Logger.recordOutput(standardPrefix + "/Odometry/OdomPeriod", inputs.OdometryPeriod);

    if (DriverStation.isDisabled() || RobotBase.isSimulation()) {
        field.setRobotPose(inputs.Pose);
    }

    // Update the swerve module states
    swerveViz.updateSwerveState(inputs);
    Logger.recordOutput(standardPrefix + "/Modules/States", inputs.ModuleStates);

    // If a loggable is configured for choreo pathing with this drivetrain we want to update the logs related to it
    if (this.ChoreoPatherLoggable != null){
      this.ChoreoPatherLoggable.updateLog(standardPrefix, inputPrefix);
    }
  }

  public void setChoreoPatherLoggable(Loggable ChoreoPather){
    this.ChoreoPatherLoggable = ChoreoPather;
  }

  // ---- Odometry updates ----
  public void resetOdometry(Pose2d pose) {
    drivetrain.resetOdometry(pose);
  }

  /**
   * Start a continuous command to apply new swerve drive requests each loop
   * @param request Supplier of SwerveRequests that will be used to command the drivetrain
   * @return The command applying the request
   */
  public Command applyRequest(Supplier<SwerveRequest> request){
    return drivetrain.continuousRequestCommand(request, this)
      .withName("SwerveDriveRequest");
  }

  /**
   * Set a control request instantaneously
   * @param request The swerve drive request to pass to the drivetrain
   */
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
  
  /**
   * Set odometry standard deviation for when the robot is DISABLED
   */
  public void configureStandardDevsForDisabled() {
    setStateStdDevs(this.configuration.kDisabledOdometryStandardDevs);
  }

  /**
   * Set odometry standard deviation for when the robot is ENABLED
   */
  public void configureStandardDevsForEnabled() {
      setStateStdDevs(this.configuration.kEnabledOdometryStandardDevs);

  }

  // Attempt to get the sim drive train
  public MapleSimSwerveDrivetrain getSimDrivetrain(){
    if (drivetrain instanceof DrivetrainSimIO){
      return ((DrivetrainSimIO) drivetrain).getMapleSimDrive();
    }

    return null;
  }
}
