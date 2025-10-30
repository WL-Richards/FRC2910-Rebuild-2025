// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.simulation.visualizations;


import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team6443.frc2025.Robot;
import com.team6443.lib.subsystems.drive.DrivetrainInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/** 
 * Visualizer for the drive base within advantage scope
 */
public class DrivetrainVisualization {
    private final String logPrefix;

    // Maxium speed of the robot in meters per second
    private final double kMaxRobotSpeed;

    // Game field reference
    private final Field2d field = new Field2d();

    // Record the last pose and the time the pose was recorded at using the AdvantageKit logger time 
    private Pose2d lastPose = Pose2d.kZero;
    private double lastPoseTime = Logger.getTimestamp();

    // Mechanisms to represent the swerve module states
    private final Mechanism2d[] moduleMechanisms = 
        new Mechanism2d[]{
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1)
        };

    // A direction and length changing ligament for speed representation
    private final MechanismLigament2d[] moduleSpeeds =
        new MechanismLigament2d[] {
            moduleMechanisms[0]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[1]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[2]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
            moduleMechanisms[3]
                    .getRoot("RootSpeed", 0.5, 0.5)
                    .append(new MechanismLigament2d("Speed", 0.5, 0)),
        };

    // A direction changing and length constant ligament for module direction
    private final MechanismLigament2d[] moduleDirections =
            new MechanismLigament2d[] {
                moduleMechanisms[0]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[1]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[2]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
                moduleMechanisms[3]
                        .getRoot("RootDirection", 0.5, 0.5)
                        .append(
                                new MechanismLigament2d(
                                        "Direction", 0.1, 0, 0, new Color8Bit(Color.kWhite))),
            };

    public DrivetrainVisualization(double maxSpeed, String logPrefix){
        this.kMaxRobotSpeed = maxSpeed;
        this.logPrefix = logPrefix;
    }
    
    public void updateViz(SwerveDriveState state){
        if (state == null || state.Pose == null || state.ModuleStates == null) {
            return;
        }

        // --- Log Robot Pose onto field
        Pose2d pose = state.Pose;
        Logger.recordOutput(logPrefix+ "/Visualizations/DrivetrainViz/Pose2D", pose);

        Pose3d pose3d =
            new Pose3d(
                    pose.getX(),
                    pose.getY(),
                    0.0,
                    new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
        Logger.recordOutput(logPrefix+ "/Visualizations/DrivetrainViz/Pose3D", pose3d);

        if (DriverStation.isDisabled() || Robot.isSimulation()) {
            field.setRobotPose(pose);
        }

        // --- Log robot speed

        // Time sync the pose and velocity before logging them
        double currentTime = Logger.getTimestamp();
        double diffTime = currentTime - lastPoseTime;
        lastPoseTime = currentTime;
        Translation2d distanceDiff = pose.minus(lastPose).getTranslation();
        lastPose = pose;
        Translation2d velocities = distanceDiff.div(diffTime);

        Logger.recordOutput("/Visualizations/DrivetrainViz/Speed", velocities.getNorm());
        Logger.recordOutput("/Visualizations/DrivetrainViz/VelocityX", velocities.getX());
        Logger.recordOutput("/Visualizations/DrivetrainViz/VelocityY", velocities.getY());
        Logger.recordOutput("/Visualizations/DrivetrainViz/OdomPeriod", state.OdometryPeriod);
        Logger.recordOutput("/Visualizations/DrivetrainViz/ModuleStatesCurrent", state.ModuleStates);

        // Update module visualizations
        for (int i = 0; i < 4; ++i) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * kMaxRobotSpeed));
        }
    }

}
