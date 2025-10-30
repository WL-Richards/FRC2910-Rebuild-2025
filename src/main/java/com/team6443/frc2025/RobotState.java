// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025;

import java.util.Map.Entry;
import java.util.concurrent.atomic.AtomicReference;

import org.littletonrobotics.junction.Logger;

import com.team6443.frc2025.constants.RobotStateConstants;
import com.team6443.lib.logging.interfaces.Loggable;
import com.team6443.lib.math.ConcurrentTimeInterpolatableBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * Robot state class that retains all information that is used to determine the robots current state.
 * This includes robot pose, velocities and mechanism positions, etc.
 * 
 * The states stored in this SHOULD ALL BE THREAD SAFE
 */
public final class RobotState implements Loggable {

    /**
     * Tracks only values as the relate to robot odometry
     */
    class Odometry {
        // Thread safe buffer to track robot pose over time this pose includes updates from vision
        public final ConcurrentTimeInterpolatableBuffer<Pose2d> TimeInterpolatableRobotPose = 
            ConcurrentTimeInterpolatableBuffer.createBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);

        /// -- Angular Velocity Time Buffers
        public final ConcurrentTimeInterpolatableBuffer<Double> DriveYawAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);
        public final ConcurrentTimeInterpolatableBuffer<Double> DriveRollAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);
        public final ConcurrentTimeInterpolatableBuffer<Double> DrivePitchAngularVelocity =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);

        /// -- Robot additional rotation time buffers
        public final ConcurrentTimeInterpolatableBuffer<Double> DrivePitchRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);
        public final ConcurrentTimeInterpolatableBuffer<Double> DriveRollRads =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);
        
        /// -- Robot Acceleration time buffers
        public final ConcurrentTimeInterpolatableBuffer<Double> DriveAccelX =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);
        public final ConcurrentTimeInterpolatableBuffer<Double> DriveAccelY =
            ConcurrentTimeInterpolatableBuffer.createDoubleBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);

        // Current robot-relative chassis speeds (measured from encoders)
        public final AtomicReference<ChassisSpeeds> actualRobotRelativeChassisSpeeds =
                new AtomicReference<>(new ChassisSpeeds());

        // Current field-relative chassis speeds (measured from encoders)
        public final AtomicReference<ChassisSpeeds> actualFieldRelativeChassisSpeeds =
                new AtomicReference<>(new ChassisSpeeds());

        // Desired robot-relative chassis speeds (set by control systems)
        public final AtomicReference<ChassisSpeeds> desiredRobotRelativeChassisSpeeds =
                new AtomicReference<>(new ChassisSpeeds());

        // Desired field-relative chassis speeds (set by control systems)
        public final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds =
                new AtomicReference<>(new ChassisSpeeds());

        // Chassis speeds with gyro rotation rate instead of module computed (measured from encoders + gyro)
        public final AtomicReference<ChassisSpeeds> gyroFusedChassisSpeeds =
                new AtomicReference<>(new ChassisSpeeds());
        
        public Odometry(){
            // Add a sample at 0,0 so the bit exists
            this.TimeInterpolatableRobotPose.addSample(0.0, Pose2d.kZero);
        }

        /**
         * Logs the latest pose in the time interpolated pose 
         * @param key Where to log the result to
         * @param buffer Buffer we are logging the data from
         */
        public static void logTimeInterpolatedPose(String key, ConcurrentTimeInterpolatableBuffer<Pose2d> buffer){
            Entry<Double, Pose2d> latest = buffer.getLatest();
            if(latest != null){
                Logger.recordOutput(key, latest.getValue());
            }
        }
    }


    /*  Singleton setup for robot state */
    private static RobotState robotState = null;
    public static RobotState get(){
        if (robotState == null){
            robotState = new RobotState();
        }
        return robotState;
    }

    /* Normal class properties */
    private final RobotState.Odometry odometryState = new RobotState.Odometry();

    public void addOdometryMeasurement(double timestamp, Pose2d pose){
        odometryState.TimeInterpolatableRobotPose.addSample(timestamp, pose);
    }

    /**
     * Add the current motion measurements for the robot to the time buffer for logging
     * @param timestamp The timestamp for which these measurements were logged
     * @param angularRollRadPerS The angular roll velocity in rads per second
     * @param angularPitchRadsPerS The angular pitch velocity in rads per second
     * @param angularYawRadPerS The angular yaw velocity in rads per second
     * @param pitchRads The current pitch in rads
     * @param rollRads The current roll in rads
     * @param accelX The x acceleration of the robot in meters per second per second
     * @param accelY The Y acceleration of the robot in meters per second per second
     * @param actualRobotRelativeChassisSpeeds The actual measurement of robot speeds from the swerve modules
     * @param actualFieldRelativeChassisSpeeds The actual measurements of the robot speed converted to field relative
     * @param desiredRobotRelativeChassisSpeeds The desired robot speeds for the swerve modules
     * @param desiredFieldRelativeChassisSpeeds The desired robot speeds for the swerve modules relative to the field
     * @param gyroFusedChassisSpeeds The actual measurement of robot speeds relative to the field using gyro rotation rates instead of modules 
     */
    public void addChassisMotionMeasurements(
        double timestamp,
        double angularRollRadPerS,
        double angularPitchRadsPerS,
        double angularYawRadPerS,
        double pitchRads,
        double rollRads,
        double accelX,
        double accelY,
        ChassisSpeeds actualRobotRelativeChassisSpeeds,
        ChassisSpeeds actualFieldRelativeChassisSpeeds,
        ChassisSpeeds desiredRobotRelativeChassisSpeeds,
        ChassisSpeeds desiredFieldRelativeChassisSpeeds,
        ChassisSpeeds gyroFusedChassisSpeeds
    ){
        // Add entries to time buffers for robot state
        odometryState.DriveRollAngularVelocity.addSample(timestamp, angularRollRadPerS);
        odometryState.DrivePitchAngularVelocity.addSample(timestamp, angularPitchRadsPerS);
        odometryState.DriveYawAngularVelocity.addSample(timestamp, angularYawRadPerS);

        odometryState.DrivePitchRads.addSample(timestamp, pitchRads);
        odometryState.DriveRollRads.addSample(timestamp, rollRads);

        odometryState.DriveAccelX.addSample(timestamp, accelX);
        odometryState.DriveAccelY.addSample(timestamp, accelY);

        // Update robot chassis speeds
        // --- Actual speeds
        odometryState.actualRobotRelativeChassisSpeeds.set(actualRobotRelativeChassisSpeeds);
        odometryState.actualFieldRelativeChassisSpeeds.set(actualFieldRelativeChassisSpeeds);

        // --- Actual speeds + Gyro Speeds
        odometryState.gyroFusedChassisSpeeds.set(gyroFusedChassisSpeeds);
        
        // --- Desired speeds
        odometryState.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
        odometryState.desiredRobotRelativeChassisSpeeds.set(desiredRobotRelativeChassisSpeeds);
    }

    // --- Loggable Implementation ---
    @Override
    public void updateLog(String prefix) {
        RobotState.Odometry.logTimeInterpolatedPose("RobotState/FinalRobotPose2d", odometryState.TimeInterpolatableRobotPose);
    }

}
