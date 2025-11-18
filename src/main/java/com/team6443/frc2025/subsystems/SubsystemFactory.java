// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.frc2025.subsystems.drive.DrivetrainIOHardware;
import com.team6443.frc2025.subsystems.drive.DrivetrainIOSim;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.frc2025.subsystems.vision.VisionSubsystem;
import com.team6443.lib.config.camera.CameraConfiguration;
import com.team6443.lib.config.camera.SimulatedCameraConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.factories.motors.TalonFXFactory;
import com.team6443.lib.motors.hardware.TalonFXIO;
import com.team6443.lib.subsystems.simulation.elevator.SimulatedElevator;
import com.team6443.lib.subsystems.simulation.elevator.SimulatedElevator.SimulatedElevatorConfiguration;
import com.team6443.lib.subsystems.vision.limelight.Limelight4IOHardware;
import com.team6443.lib.subsystems.vision.limelight.LimelightIOSim;

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
                            RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
                            RobotRuntimeConstants.kRobotConfiguration.getSwerveConfigurations()
                        )
                )
                .withStartingPose(new Pose2d(2.5, 4, Rotation2d.fromDegrees(0)));

            // ---- Physical instance of drivetrain ----
            case REPLAY: // fall down to default
            case REAL:
            default:
                return new DrivetrainSubsystem(
                    RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
                    new DrivetrainIOHardware(
                            RobotRuntimeConstants.kRobotConfiguration.getDrivetrainConfiguration(),
                            RobotRuntimeConstants.kRobotConfiguration.getSwerveConfigurations()
                        )
                );
        }

    }

    public static VisionSubsystem createVisionSubsystem(){
        switch (RobotRuntimeConstants.kCurrentRuntimeMode) {
            // ---- Simulation instance of drivetrain ----
            case SIM:
                return new VisionSubsystem(
                    new LimelightIOSim(RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(0)),
                    new LimelightIOSim(RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(1)),
                    new LimelightIOSim(RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(2)),
                    new LimelightIOSim(RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(3))
                );
                

            // ---- Physical instance of drivetrain ----
            case REPLAY: // fall down to default
            case REAL:
            default:
                return new VisionSubsystem(
                    new Limelight4IOHardware(RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(0)),
                    new Limelight4IOHardware(RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(1)),
                    new Limelight4IOHardware(RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(2)),
                    new Limelight4IOHardware(RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(3))
                );
        }
    }
}
