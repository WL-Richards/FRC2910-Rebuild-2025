// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team6443.lib.config.autonomous.ChoreoPathingConfiguration;

import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** 
 * Handle Choreo path planning for a given robot
*/
public class ChoreoPathing {
    private final PIDController xTranslationController;
    private final PIDController yTranslationController;
    private final PIDController yawRotationController;

    private AutoFactory autoFactory;

    public ChoreoPathing(
        ChoreoPathingConfiguration configuration
    ){
        xTranslationController = configuration.kXTranslationConfiguration.generateController();
        yTranslationController = configuration.kYTranslationConfiguration.generateController();

        yawRotationController = configuration.kYawRotationConfiguration.generateController();
    }

    public ChoreoPathing withAutoFactory(
        Supplier<Pose2d> robotPoseSupplier,
        Consumer<Pose2d> resetOdometry,
        Consumer<SwerveRequest> consumeDriveTrainRequest,
        boolean allianceFlipping,
        Subsystem drivetrainSubsystem
    ){
        autoFactory = new AutoFactory(
            robotPoseSupplier, 
            resetOdometry, 
            (SwerveSample sample) -> consumeDriveTrainRequest.accept(
                new SwerveRequest.ApplyFieldSpeeds()
                    .withSpeeds(computeSpeeds(sample, robotPoseSupplier.get()))
                    .withWheelForceFeedforwardsX(sample.moduleForcesX())
                    .withWheelForceFeedforwardsY(sample.moduleForcesY())
                    .withDriveRequestType(SwerveModule.DriveRequestType.Velocity)
            ), 
            allianceFlipping, 
            drivetrainSubsystem);
        return this;
    }

    public AutoFactory getAutoFactory(){
        return autoFactory;
    }

    private ChassisSpeeds computeSpeeds(SwerveSample sample, Pose2d pose){
        // Generate the next speeds for the robot
        return new ChassisSpeeds(
            sample.vx + xTranslationController.calculate(pose.getX(), sample.x),
            sample.vy + yTranslationController.calculate(pose.getY(), sample.y),
            sample.omega + yawRotationController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
    }
}
