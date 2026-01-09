// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.drive.io;

import java.util.List;
import java.util.function.Consumer;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.frc2025.state.SimulatedRobotState;
import com.team6443.lib.config.subsystems.drive.DrivetrainConfiguration;
import com.team6443.lib.config.subsystems.drive.simulation.DrivetrainSimConfiguration;
import com.team6443.lib.config.swerve.SwerveModuleConfiguration;
import com.team6443.lib.subsystems.drive.DrivetrainInputs;
import com.team6443.lib.subsystems.drive.simulation.MapleSimSwerveDrivetrain;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

/**
 * The {@code DrivetrainIOSim} class extends {@link DrivetrainHardwareIO} to provide simulation-specific
 * functionality for the swerve drive system. It integrates with WPILib's simulation framework for
 * testing and development.
 */
public class DrivetrainSimIO extends DrivetrainHardwareIO {

    // Simulation helpers
    private Notifier simulationThread = null;        

    // Simulation configuration properties of the drive train, things like sim update rate
    private final DrivetrainSimConfiguration simConfig; // 5 ms
    public MapleSimSwerveDrivetrain drivetrainSim = null;
    private SwerveModuleConstants<?, ?, ?>[] moduleConstants;
    private List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> moduleConfigurations;

    // Update the swerve drive state for the simulation
    private Consumer<SwerveDriveState> simSwerveStateConsumer =
        state -> {
            if(drivetrainSim != null){
                state.Pose = drivetrainSim.mapleSimSwerveDrivetrain.getSimulatedDriveTrainPose();
            }
            SimulatedRobotState.get().addOdometryMeasurement(state.Pose);
            swerveTelemetryConsumer.accept(state);
        };


    public DrivetrainSimIO(
        DrivetrainSimConfiguration simConfig,
        DrivetrainConfiguration driveTrainConfiguration,
        List<SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>> swerveModuleConfiguration
    ){
        // regulation occurs in-place
        super(MapleSimSwerveDrivetrain.regulateModuleConstantForSimulation(driveTrainConfiguration), swerveModuleConfiguration);

        this.simConfig = simConfig;
        this.moduleConstants = driveTrainConfiguration.kModuleConstants;
        this.moduleConfigurations = swerveModuleConfiguration;

        registerTelemetry(simSwerveStateConsumer);
        startSimThread();
    }

    /**
     * Start the simulation thread for the maple sim drive train
     */
    public void startSimThread() {
        
        drivetrainSim =
                new MapleSimSwerveDrivetrain(
                        Units.Seconds.of(simConfig.kSimLoopPeriodS),                               // Simulation Update Rate 0.005s = 5ms = 200hz
                        Units.Pounds.of(simConfig.kPhysicalConfiguration.kRobotWeightPounds),       // Weight of the robot in pounds
                        Units.Meters.of(simConfig.kPhysicalConfiguration.kBumperWidthMeters),       // Bumper width meters
                        Units.Meters.of(simConfig.kPhysicalConfiguration.kBumperLengthMeters),      // Bumper length meters
                        DCMotor.getKrakenX60(simConfig.kModuleDriveMotorCount),                     // Number of drive motors on 1 swerve module
                        DCMotor.getKrakenX60(simConfig.kModuleSteerMotorCount),                     // Number of steer motors on 1 swerve module
                        simConfig.kPhysicalConfiguration.kWheelCoefficientOfFriction,               // Wheel coef. of friction (its ability to resist movement)
                        getModuleLocations(),                                                       // Translation 2Ds representing the location of each module
                        getPigeon2(),                                                               // Get the pigeon 2 used by the drive train
                        getModules(),                                                               // Get the representation of the swerve modules themselves
                        moduleConstants,                                                            // Get the swerve module constants values
                        moduleConfigurations);                                                      // List of configurations of the swerve modules
        new SwerveDriveKinematics(getModuleLocations());
        // Create and start simulation thread
        simulationThread = new Notifier(drivetrainSim::update);
        simulationThread.setName("DrivetrainSimNotifier");
        simulationThread.startPeriodic(simConfig.kSimLoopPeriodS);
    }

    @Override
    public SwerveDriveKinematics getSwerveKinematics() {
        return new SwerveDriveKinematics(getModuleLocations());
    }

    /**
     * Handles resetting the odometry position, if we have a valid drivetrain sim its world pose will be reset
     * @param pose The pose of which we want 0,0 to now be
     */
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
    }

    /**
     * Get a reference to the underlying maple sim drive train
     * @return The MapleSimSwerveDrivetrain running the simulation
     */
    public MapleSimSwerveDrivetrain getMapleSimDrive() {
        return drivetrainSim;
    }

}
