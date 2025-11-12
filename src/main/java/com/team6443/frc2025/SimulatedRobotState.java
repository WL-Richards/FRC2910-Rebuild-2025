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
import edu.wpi.first.wpilibj.Timer;

/**
 * Robot state class that retains all information that is used to determine the robots current state.
 * This includes robot pose, velocities and mechanism positions, etc.
 * 
 * The states stored in this SHOULD ALL BE THREAD SAFE
 */
public final class SimulatedRobotState implements Loggable {

    /**
     * Tracks only values as the relate to robot odometry
     */
    class Odometry {
        // Thread safe buffer to track robot pose over time this pose includes updates from vision
        public final ConcurrentTimeInterpolatableBuffer<Pose2d> TimeInterpolatableSimulatedRobotPose = 
            ConcurrentTimeInterpolatableBuffer.createBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);

        public Odometry(){}

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
    private static SimulatedRobotState robotState = null;
    public static SimulatedRobotState get(){
        if (robotState == null){
            robotState = new SimulatedRobotState();
        }
        return robotState;
    }

    /* Normal class properties */
    private final SimulatedRobotState.Odometry odometryState = new SimulatedRobotState.Odometry();

    public synchronized void addOdometryMeasurement(Pose2d pose){
        odometryState.TimeInterpolatableSimulatedRobotPose.addSample(Timer.getFPGATimestamp(), pose);
    }

    /**
     * Retrieve the latest robot field pose from the simulated robot
     * @return Simulated robot field pose
     */
    public Pose2d getLatestFieldRobotPose(){
        var entry = odometryState.TimeInterpolatableSimulatedRobotPose.getInternalBuffer().lastEntry();
        if(entry == null){
            return null;
        }
        return entry.getValue();
    }

    // --- Loggable Implementation ---
    @Override
    public void updateLog(String prefix) {
        RobotState.Odometry.logTimeInterpolatedPose("RobotState/SimulatedRobotPose2d", odometryState.TimeInterpolatableSimulatedRobotPose);
    }

}
