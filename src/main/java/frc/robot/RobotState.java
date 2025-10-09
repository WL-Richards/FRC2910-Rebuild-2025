// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map.Entry;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.RobotStateConstants;
import frc.robot.util.interfaces.logging.Loggable;
import frc.robot.util.math.ConcurrentTimeInterpolatableBuffer;

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
    private class Odometry {
        // Thread safe buffer to track robot pose over time this pose includes updates from vision
        public final ConcurrentTimeInterpolatableBuffer<Pose2d> TimeInterpolatableRobotPose = 
            ConcurrentTimeInterpolatableBuffer.createBuffer(RobotStateConstants.Kinematics.kRobotPoseWindowLengthSeconds);

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
    public final RobotState.Odometry OdometryState = new RobotState.Odometry();

    // --- Loggable Implementation ---
    @Override
    public void updateLog() {
        RobotState.Odometry.logTimeInterpolatedPose("RobotState/FinalRobotPose2d", OdometryState.TimeInterpolatableRobotPose);
    }

    
}
