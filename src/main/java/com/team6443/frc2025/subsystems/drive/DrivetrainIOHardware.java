// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.drive;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.CANBus.CANBusStatus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team6443.lib.RobotState;
import com.team6443.lib.can.CANStatusLogger;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.config.talonFX.TalonFXConfigEquality;
import com.team6443.lib.logging.interfaces.Loggable;
import com.team6443.lib.subsystems.drive.DrivetrainIO;
import com.team6443.lib.subsystems.drive.DrivetrainInputs;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** 
 * Hardware implementation of the drivetrain
 */
public class DrivetrainIOHardware extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements DrivetrainIO {

    // Create a thread safe cached version of the telemetry that we can use to produce logs from
    private AtomicReference<SwerveDriveState> swerveTelemetryCache = new AtomicReference<>();

    // Updates the odometry information from the drive train within our overall robot state as well as updating the cache 
    protected Consumer<SwerveDriveState> swerveTelemetryConsumer =
        state -> {

            // Update the state via deep-copy
            swerveTelemetryCache.set(state.clone());
            
            RobotState.get().addOdometryMeasurement(
                // Synchronize the RoboRIO clock with the system time and return the rio time of the state 
                (Timer.getFPGATimestamp() - Utils.getCurrentTimeSeconds()) + state.Timestamp,

                // New pose of the swerve drive
                state.Pose
            );
        };

    // ------ Pigeon2 signals ------

    // --- Angular Velocities
    private final StatusSignal<AngularVelocity> angularPitchVelocity;
    private final StatusSignal<AngularVelocity> angularRollVelocity;
    private final StatusSignal<AngularVelocity> angularYawVelocity;

    // --- Angular Rotations
    private final StatusSignal<Angle> roll;
    private final StatusSignal<Angle> pitch;

    // --- Accelerations
    private final StatusSignal<LinearAcceleration> accelerationX;
    private final StatusSignal<LinearAcceleration> accelerationY;
    Matrix<N3, N1> stateStdDevs = null;

    public DrivetrainIOHardware(
        DrivetrainConfiguration driveTrainConfiguration,
        List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> swerveModuleConfiguration
    ){

        // Create the CTRE swerve drive train from our robot configuration
        super(
            TalonFX::new, 
            TalonFX::new, 
            CANcoder::new, 
            driveTrainConfiguration.kDriveConstants, 
            250.0, 
            driveTrainConfiguration.kModuleConstants
        );

        // Retrieve all Pigeon2 signals
        angularPitchVelocity = getPigeon2().getAngularVelocityYWorld();
        angularRollVelocity = getPigeon2().getAngularVelocityXWorld();
        angularYawVelocity = getPigeon2().getAngularVelocityZWorld();
        roll = getPigeon2().getRoll();
        pitch = getPigeon2().getPitch();
        accelerationX = getPigeon2().getAccelerationX();
        accelerationY = getPigeon2().getAccelerationY();

        // Set yaw velocity to update at 250 hz, we care more about this value
        BaseStatusSignal.setUpdateFrequencyForAll(250, angularYawVelocity);

        // Set rest to update at 100
        BaseStatusSignal.setUpdateFrequencyForAll(
            100,
            angularPitchVelocity,
            angularRollVelocity,
            roll,
            pitch,
            accelerationX,
            accelerationY
        );

        // Register the drivetrain with the CAN status logger
        CANStatusLogger.get(driveTrainConfiguration.kDriveConstants.CANBusName).registerSwerveDrivetrain(
            this,
            swerveModuleConfiguration,
            driveTrainConfiguration.kGyroDeviceID
        );

        // Set odometry thread to have the highest priority
        this.getOdometryThread().setThreadPriority(99);

        // Register the telemetry object with the CTRE swerve drive train
        registerTelemetry(swerveTelemetryConsumer);
        
    }

    @Override
    public void updateInputs(DrivetrainInputs inputs) {
        // If we can't get the reference then just return and we will try again next loop
        if(swerveTelemetryCache.get() == null) return;
        inputs.fromSwerveDriveState(swerveTelemetryCache.get());

        // Get the top down rotation of the robot
        Rotation2d gyroRotation = inputs.Pose.getRotation();
        inputs.gyroYawAngle = gyroRotation.getDegrees();

        // --- True robot chassis speeds (or atleast true to what it knows)
        // Derive the robots chassis speed from the state of each of the swerve modules within the drive train
        ChassisSpeeds actualRobotRelativeChassisSpeeds = getKinematics().toChassisSpeeds(inputs.ModuleStates);

        // Convert our robot relative speeds into speeds relative to the game field relative speeds
        ChassisSpeeds actualFieldRelativeChassisSpeeds = ChassisSpeeds
            .fromRobotRelativeSpeeds(
                actualRobotRelativeChassisSpeeds, 
                gyroRotation
            );
        
        // --- Desired chassis speeds based on input
        ChassisSpeeds desiredRobotRelativeChassisSpeeds = getKinematics().toChassisSpeeds(inputs.ModuleTargets);
        ChassisSpeeds desiredFieldRelativeChassisSpeeds = ChassisSpeeds
            .fromRobotRelativeSpeeds(
                desiredRobotRelativeChassisSpeeds, 
                gyroRotation
            );
        
        // Update all gyro signals
        BaseStatusSignal.refreshAll(
            angularRollVelocity,
            angularPitchVelocity,
            angularYawVelocity,
            pitch,
            roll,
            accelerationX,
            accelerationY
        );

        // ------ Update Robot State ------
        double timestamp = Timer.getFPGATimestamp();
        double rollRadsPerS = Units.degreesToRadians(angularRollVelocity.getValueAsDouble());
        double pitchRadsPerS = Units.degreesToRadians(angularPitchVelocity.getValueAsDouble());
        double yawRadsPerS = Units.degreesToRadians(angularYawVelocity.getValueAsDouble());

        double pitchRads = Units.degreesToRadians(pitch.getValueAsDouble());
        double rollRads = Units.degreesToRadians(roll.getValueAsDouble());

        double accelX = accelerationX.getValueAsDouble();
        double accelY = accelerationY.getValueAsDouble();
        
        // Only use the translational information from the odometry because the gyro is more trustworthy for rotational rate
        ChassisSpeeds gyroFusedFieldRelativeChassisSpeeds = new ChassisSpeeds(
            actualFieldRelativeChassisSpeeds.vxMetersPerSecond,
            actualFieldRelativeChassisSpeeds.vyMetersPerSecond,
            yawRadsPerS
        );

        // Add all the speed measurements to our robot state
        RobotState.get().addChassisMotionMeasurements(
            timestamp, 
            rollRadsPerS, 
            pitchRadsPerS, 
            yawRadsPerS, 
            pitchRads, 
            rollRads, 
            accelX, 
            accelY, 
            actualRobotRelativeChassisSpeeds, 
            actualFieldRelativeChassisSpeeds, 
            desiredRobotRelativeChassisSpeeds, 
            desiredFieldRelativeChassisSpeeds, 
            gyroFusedFieldRelativeChassisSpeeds
        );

    }

    @Override
    public void resetOdometry(Pose2d pose) {
        super.resetPose(pose);
    }

    @Override
    public void logModules(SwerveDriveState state,  String prefix) {
        final String[] moduleNames = {prefix + "/Modules/FL/", prefix + "/Modules/FR/", prefix + "/Modules/BL/", prefix + "/Modules/BR/"};
        if (state.ModuleStates == null) return;
        for (int i = 0; i < getModules().length; i++) {
            Logger.recordOutput(
                    moduleNames[i] + " Absolute Encoder Angle",
                    getModule(i).getEncoder().getAbsolutePosition().getValueAsDouble() * 360);
            Logger.recordOutput(
                    moduleNames[i] + " Steering Angle", state.ModuleStates[i].angle);
            Logger.recordOutput(
                    moduleNames[i] + " Target Steering Angle", state.ModuleTargets[i].angle);
            Logger.recordOutput(
                    moduleNames[i] + " Drive Velocity",
                    state.ModuleStates[i].speedMetersPerSecond);
            Logger.recordOutput(
                    moduleNames[i] + " Target Drive Velocity",
                    state.ModuleTargets[i].speedMetersPerSecond);
        }
    }

    @Override
    public void setControl(SwerveRequest request) {
        super.setControl(request);
    }
   
    @Override
    public Command continuousRequestCommand(Supplier<SwerveRequest> requestSupplier, Subsystem... subsystemsRequired) {
        return Commands.run(() -> this.setControl(requestSupplier.get()), subsystemsRequired);
    }

    @Override
    public void setOdometryStdDevs(double xStd, double yStd, double rotStd) {

        // Don't run fill more than once no need to run the garbage collector all the time
        if (stateStdDevs == null) {
            stateStdDevs = VecBuilder.fill(xStd, yStd, rotStd);
        } else {
            stateStdDevs.set(0, 0, xStd);
            stateStdDevs.set(1, 0, yStd);
            stateStdDevs.set(2, 0, rotStd);
        }
        
        this.setStateStdDevs(stateStdDevs);
    }
}
