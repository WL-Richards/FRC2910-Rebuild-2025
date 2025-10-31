// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.google.flatbuffers.Constants;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.frc2025.subsystems.drive.DrivetrainIOHardware;
import com.team6443.frc2025.subsystems.drive.DrivetrainIOSim;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.factories.motors.TalonFXFactory;
import com.team6443.lib.motors.hardware.TalonFXIO;
import com.team6443.lib.subsystems.simulation.drive.MapleSimSwerveDrivetrain;
import com.team6443.lib.subsystems.simulation.elevator.SimulatedElevator;
import com.team6443.lib.subsystems.simulation.elevator.SimulatedElevator.SimulatedElevatorConfiguration;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * High level subsystem factories intended to be called from RobotContainer when the robot Subsystem representation is being constructed
 */
public class SubsystemFactory {


    /**
     * Build the elevator subsystem factor that will be used
     * @return The elevator subsystem being used in this instance
     */
    public static ElevatorSubsystem createElevatorSubsystem(){
        ElevatorSubsystem elevator = null;
        ServoMotorFollowerConfiguration<TalonFXConfiguration> elevatorConfig = RobotRuntimeConstants.kRobotConfiguration.getElevatorConfiguration();
        SimulatedElevatorConfiguration simulatedElevatorConfig  = RobotRuntimeConstants.kRobotConfiguration.getSimulatedElevatorConfiguration();
       

        // Build the correct instance of the elevator subsystem
        switch (RobotRuntimeConstants.kCurrentRuntimeMode) {
            // ---- Simulation instance of elevator ----
            case SIM:
                SimulatedElevator simElevator = new SimulatedElevator(elevatorConfig, simulatedElevatorConfig);
                elevator = new ElevatorSubsystem(
                    elevatorConfig,
                    simElevator.getLeadTalon(), 
                    simElevator.getFollowerTalons()
                );
                break;

            // ---- Physical instance of elevator ----
            case REPLAY: // fall down to default
            case REAL:
            default:
                elevator = new ElevatorSubsystem(
                    elevatorConfig,
                    TalonFXFactory.createIO(elevatorConfig), 
                    new TalonFXIO[] {
                        TalonFXFactory.createIO(elevatorConfig.followerConfigurations.get(0).config)
                    }
                );
                break;
        }

        return elevator;
    }
    

    public static DrivetrainSubsystem createDrivetrainSubsystem(){

        switch (RobotRuntimeConstants.kCurrentRuntimeMode) {
            // ---- Simulation instance of drivetrain ----
            case SIM:
                return new DrivetrainSubsystem(
                        RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
                        new DrivetrainIOSim(
                            RobotRuntimeConstants.kRobotConfiguration.getSimulatedDrivetrainConfiguration(),
                            RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration().kDriveConstants,
                            // NOTE: regulateModuleConstantsForSimulation this must be run to prevent modules from doing cursed things
                            MapleSimSwerveDrivetrain.regulateModuleConstantsForSimulation(RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration().kModuleConstants)
                        )
                )
                .withStartingPose(new Pose2d(3, 3, new Rotation2d(0.1)));

            // ---- Physical instance of drivetrain ----
            case REPLAY: // fall down to default
            case REAL:
            default:
                return new DrivetrainSubsystem(
                    RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
                    new DrivetrainIOHardware(
                            RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration().kDriveConstants,
                            RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration().kModuleConstants
                        )
                );
        }

    }
}
