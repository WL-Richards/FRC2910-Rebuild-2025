// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.simulation.visualizations;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
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

    // ----- Swerve Drive Module Visualization -----
    // Mechanisms to represent the swerve module states
    private final Mechanism2d[] moduleMechanisms = new Mechanism2d[] {
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1),
            new Mechanism2d(1, 1)
    };

    // A direction and length changing ligament for speed representation
    private final MechanismLigament2d[] moduleSpeeds = new MechanismLigament2d[] {
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
    private final MechanismLigament2d[] moduleDirections = new MechanismLigament2d[] {
            moduleMechanisms[0]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
            moduleMechanisms[1]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
            moduleMechanisms[2]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
            moduleMechanisms[3]
                    .getRoot("RootDirection", 0.5, 0.5)
                    .append(
                            new MechanismLigament2d(
                                    "Direction", 0.1, 0, 0,
                                    new Color8Bit(Color.kWhite))),
    };
    // ---------------------------------------------

    public DrivetrainVisualization(double maxSpeed, String logPrefix) {
        this.kMaxRobotSpeed = maxSpeed;
        this.logPrefix = logPrefix;
    }

    public void updateViz(SwerveDriveState state) {
        if (state == null || state.Pose == null || state.ModuleStates == null) {
            return;
        }

        // --- Log Robot Pose onto field ---
        Pose2d pose = state.Pose;
        Logger.recordOutput(logPrefix + "/Visualizations/DrivetrainViz/Pose2D", pose);

        Pose3d pose3d = new Pose3d(
                pose.getX(),
                pose.getY(),
                0.0,
                new Rotation3d(0.0, 0.0, pose.getRotation().getRadians()));
        Logger.recordOutput(logPrefix + "/Visualizations/DrivetrainViz/Pose3D", pose3d);

        if (DriverStation.isDisabled() || RobotBase.isSimulation()) {
            field.setRobotPose(pose);
        }

        // --- Log robot speed ---
        Logger.recordOutput("/Visualizations/DrivetrainViz/Speed", new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond).getNorm());
        Logger.recordOutput("/Visualizations/DrivetrainViz/VelocityX", state.Speeds.vxMetersPerSecond);
        Logger.recordOutput("/Visualizations/DrivetrainViz/VelocityY", state.Speeds.vyMetersPerSecond);
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
