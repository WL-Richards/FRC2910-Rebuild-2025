// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.state;

import java.util.ArrayList;
import java.util.List;
import java.util.Map.Entry;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.team6443.frc2025.Robot;
import com.team6443.lib.constants.RobotStateConstants;
import com.team6443.lib.core.logging.Loggable;
import com.team6443.lib.math.ConcurrentTimeInterpolatableBuffer;

import com.team6443.lib.subsystems.vision.LimelightVisionInputs;
import com.team6443.lib.subsystems.vision.util.VisionFieldPoseEstimate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

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
        public final ConcurrentTimeInterpolatableBuffer<Pose2d> TimeInterpolatableEstimatedRobotPose = 
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
            this.TimeInterpolatableEstimatedRobotPose.addSample(0.0, Pose2d.kZero);
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

        /**
         * Logs the latest pose in the time interpolated pose 
         * @param key Where to log the result to
         * @param buffer Buffer we are logging the data from
         */
        public static void logTimeInterpolatedPose3d(String key, ConcurrentTimeInterpolatableBuffer<Pose2d> buffer){
            Entry<Double, Pose2d> latest = buffer.getLatest();
            if(latest != null){
                Pose2d pose = latest.getValue();
                Logger.recordOutput(key, new Pose3d(
                    pose.getX(),
                    pose.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, pose.getRotation().getRadians())));
            }
        }
    }

    /**
     * Tracks the current state of the robot vision systems
     */
    class Vision {
        public AtomicReference<Double> LastUsedMegatagTimestamp = new AtomicReference<Double>(0.0);
        public final ConcurrentTimeInterpolatableBuffer<Pose2d> TimeInterpolatableMegatagPose = ConcurrentTimeInterpolatableBuffer.createBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);
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

    private final RobotState.Vision visionState = new RobotState.Vision();

    // --- Odometry Functionality ---

    public void addOdometryMeasurement(double timestamp, Pose2d pose){
        odometryState.TimeInterpolatableEstimatedRobotPose.addSample(timestamp, pose);
    }

    /**
     * Retrieve the latest robot field pose from the simulated robot
     * @return Simulated robot field pose
     */
    public Pose2d getLatestFieldRobotPose(){
        var entry = odometryState.TimeInterpolatableEstimatedRobotPose.getInternalBuffer().lastEntry();
        if(entry == null){
            return null;
        }
        return entry.getValue();
    }
    
    /**
     * Get the field robot pose of the robot at some timestamp within the kRobotPoseWindowLengthSeconds
     * @param timestamp Timestamp to lookup
     * @return Optional Pose2d representing the robots field position
     */
    public Optional<Pose2d> getFieldRobotPose(double timestamp){
        return odometryState.TimeInterpolatableEstimatedRobotPose.getSample(timestamp);
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

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return odometryState.actualFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return odometryState.actualRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredRobotRelativeChassisSpeeds() {
        return odometryState.actualRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return odometryState.actualFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return odometryState.gyroFusedChassisSpeeds.get();
    }

    public Optional<Double> getMaxDriveYawSpeedInRange(
            double minTime, double maxTime) {
        // Gyro yaw rate not set in sim.
        if (Robot.isReal()) return getMaxAbsValueInRange(odometryState.DriveYawAngularVelocity, minTime, maxTime);
        return Optional.of(Math.abs(odometryState.actualRobotRelativeChassisSpeeds.get().omegaRadiansPerSecond));
    }

    private Optional<Double> getMaxAbsValueInRange(
            ConcurrentTimeInterpolatableBuffer<Double> buffer, double minTime, double maxTime) {
        var submap = buffer.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty()) return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get())) return max;
        else return min;
    }

    // --- Vision Functionality ---

    private Consumer<VisionFieldPoseEstimate> drivetrainVisionEstimateConsumer = null;

    public void registerDriveTrainVisionEstimateConsumer(Consumer<VisionFieldPoseEstimate> addVisionEstimateConsumer){
        this.drivetrainVisionEstimateConsumer = addVisionEstimateConsumer;
    }

    public void addMegatagEstimateMeasurement(VisionFieldPoseEstimate megatagEstimate){
        visionState.TimeInterpolatableMegatagPose.addSample(megatagEstimate.getTimestampSeconds(), megatagEstimate.getVisionRobotPoseMeters());
        visionState.LastUsedMegatagTimestamp.set(megatagEstimate.getTimestampSeconds());
        drivetrainVisionEstimateConsumer.accept(megatagEstimate);
    }

    public double getLastUsedMegatagTimestamp(){
        return visionState.LastUsedMegatagTimestamp.get();
    }

    public Pose2d getLatestMegatagEstimatedPose(){
        var entry = visionState.TimeInterpolatableMegatagPose.getInternalBuffer().lastEntry();
        if(entry == null){
            return null;
        }
    
        return entry.getValue();
    }

    // --- Loggable Implementation ---
    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        RobotState.Odometry.logTimeInterpolatedPose("SensorRobotState/RobotPose2d", odometryState.TimeInterpolatableEstimatedRobotPose);
        RobotState.Odometry.logTimeInterpolatedPose("SensorRobotState/RobotPose3d", odometryState.TimeInterpolatableEstimatedRobotPose);
    }

}
