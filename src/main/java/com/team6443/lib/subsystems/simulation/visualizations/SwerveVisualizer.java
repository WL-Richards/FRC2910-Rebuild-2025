// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.simulation.visualizations;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Visualizer for the drive base within advantage scope
 */
public class SwerveVisualizer {


    // Maxium speed of the robot in meters per second
    private final double kMaxRobotSpeed;
    
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

    public SwerveVisualizer(double maxSpeed) {
        this.kMaxRobotSpeed = maxSpeed;
    }

    public void updateSwerveState(SwerveDriveState state) {
        if (state == null || state.Pose == null || state.ModuleStates == null) {
            return;
        }

        // Update module visualizations
        for (int i = 0; i < 4; ++i) {
            moduleSpeeds[i].setAngle(state.ModuleStates[i].angle);
            moduleDirections[i].setAngle(state.ModuleStates[i].angle);
            moduleSpeeds[i].setLength(state.ModuleStates[i].speedMetersPerSecond / (2 * kMaxRobotSpeed));
        }
    }
}
