// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.autonomous;

import java.util.function.Consumer;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team6443.lib.config.autonomous.ChoreoPathingConfiguration;
import com.team6443.lib.logging.interfaces.Loggable;

import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Subsystem;

/** 
 * Handle Choreo path planning for a given robot
*/
public class ChoreoPathing implements Loggable {
    private final PIDController xTranslationController;
    private final PIDController yTranslationController;
    private final PIDController yawRotationController;

    private AutoFactory autoFactory;

    // The trajectory that is currently being run on the bot
    private AutoTrajectory currentRunningTrajectory = null;

    // --- Log info ---
    private SwerveSample latestSample = null;

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

        // Create new auto factory that will command our drivetrian's swerve modules
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
        this.latestSample = sample;
        return new ChassisSpeeds(
            sample.vx + xTranslationController.calculate(pose.getX(), sample.x),
            sample.vy + yTranslationController.calculate(pose.getY(), sample.y),
            sample.omega + yawRotationController.calculate(pose.getRotation().getRadians(), sample.heading)
        );
    }

    public void setActiveTrajectory(AutoTrajectory trajectory){
        this.currentRunningTrajectory = trajectory;
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        if(latestSample != null){
            Logger.recordOutput(standardPrefix + "/Choreo/DesiredPose", latestSample.getPose());
            Logger.recordOutput(
                    standardPrefix + "/Choreo/DesiredChassisSpeeds", latestSample.getChassisSpeeds());
            Logger.recordOutput(standardPrefix + "/Choreo/DesiredModuleForcesX", latestSample.moduleForcesX());
            Logger.recordOutput(standardPrefix + "/Choreo/DesiredModuleForcesY", latestSample.moduleForcesY());
        }

        // Log the current running trajecroy if it exists
        if(currentRunningTrajectory != null){
            Logger.recordOutput(standardPrefix + "/Choreo/TrajectoryName", currentRunningTrajectory.getRawTrajectory().name());
            Logger.recordOutput(standardPrefix + "/Choreo/DesiredTrajectory", 

            // Handle flipping the trajectory depending on the alliance
            DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue 
                ? currentRunningTrajectory.getRawTrajectory().getPoses() 
                : currentRunningTrajectory.getRawTrajectory().flipped().getPoses()
            );
        }
    }
}
