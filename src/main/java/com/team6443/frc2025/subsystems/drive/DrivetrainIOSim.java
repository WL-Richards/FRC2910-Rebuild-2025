// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.drive;

import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.google.flatbuffers.Constants;
import com.team6443.frc2025.SimulatedRobotState;
import com.team6443.lib.config.subsystems.drive.DrivetrainSimConfiguration;
import com.team6443.lib.subsystems.drive.DrivetrainInputs;
import com.team6443.lib.subsystems.simulation.drive.MapleSimSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;

/**
 * The {@code DrivetrainIOSim} class extends {@link DrivetrainIOHardware} to provide simulation-specific
 * functionality for the swerve drive system. It integrates with WPILib's simulation framework for
 * testing and development.
 */
public class DrivetrainIOSim extends DrivetrainIOHardware {
    // What to prepend to logs from this subsystem
    private String logPrefix;

    // Simulation helpers
    private Notifier simNotifier = null;        // Thread to run sim in

    private final DrivetrainSimConfiguration simConfig; // 5 ms
    public MapleSimSwerveDrivetrain drivetrainSim = null;
    private SwerveModuleConstants<?, ?, ?>[] moduleConstants;

    // Update the swerve drive state for the simulation
    private Consumer<SwerveDriveState> simSwerveStateConsumer =
        state -> {
            if(drivetrainSim != null){
                state.Pose = drivetrainSim.mapleSimSwerveDrivetrain.getSimulatedDriveTrainPose();
            }
            SimulatedRobotState.get().addOdometryMeasurement(state.Pose);
            swerveTelemetryConsumer.accept(state);
        };


    public DrivetrainIOSim(
        DrivetrainSimConfiguration simConfig,
        SwerveDrivetrainConstants  drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>[] moduleConstants
    ){
        super(drivetrainConstants, moduleConstants);
       
        this.simConfig = simConfig;
        this.moduleConstants = moduleConstants;

        registerTelemetry(simSwerveStateConsumer);
        startSimThread();
    }

    /**
     * Start the simulation thread for the maple sim drive train
     */
    public void startSimThread() {
        
        drivetrainSim =
                new MapleSimSwerveDrivetrain(
                        Units.Seconds.of(simConfig.kSimLoopPeriodMS),
                        Units.Pounds.of(simConfig.kRobotWeightPounds),
                        Units.Inches.of(simConfig.kBumperWidthInches),
                        Units.Inches.of(simConfig.kBumperLengthInches),
                        DCMotor.getKrakenX60(simConfig.kDriveMotorCount),
                        DCMotor.getKrakenX60(simConfig.kSteerMotorCount),
                        simConfig.kWheelCoefficientOfFriction,
                        getModuleLocations(),
                        getPigeon2(),
                        getModules(),
                        moduleConstants);

        simNotifier = new Notifier(drivetrainSim::update);
        simNotifier.setName("DrivetrainSimNotifier");
        simNotifier.startPeriodic(simConfig.kSimLoopPeriodMS);
    }

    public void resetOdometry(Pose2d pose){
        if(drivetrainSim != null){
            drivetrainSim.mapleSimSwerveDrivetrain.setSimulationWorldPose(pose);
            Timer.delay(0.05); // Wait one loop
        }
        super.resetOdometry(pose);
    }

    @Override
    public void updateInputs(DrivetrainInputs inputs) {
        super.updateInputs(inputs);
        Pose2d pose = SimulatedRobotState.get().getLatestFieldRobotPose();
        if(pose != null){
            Logger.recordOutput(this.logPrefix + "/Viz/SimPose", pose);
        }
    }

    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return drivetrainSim;
    }

    @Override
    public void setLoggingPrefix(String prefix) {
        this.logPrefix = prefix + "/IO/Sim";
    }

}
