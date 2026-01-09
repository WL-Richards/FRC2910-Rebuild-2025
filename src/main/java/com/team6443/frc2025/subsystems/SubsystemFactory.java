// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.frc2025.constants.field.Field2025;
import com.team6443.frc2025.state.RobotState;
import com.team6443.frc2025.state.SimulatedRobotState;
import com.team6443.frc2025.subsystems.drive.DrivetrainSubsystem;
import com.team6443.frc2025.subsystems.drive.io.DrivetrainHardwareIO;
import com.team6443.frc2025.subsystems.drive.io.DrivetrainSimIO;

import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.frc2025.subsystems.elevator.io.ElevatorHardwareIO;
import com.team6443.frc2025.subsystems.elevator.io.ElevatorSimIO;

import com.team6443.frc2025.subsystems.vision.VisionSubsystem;
import com.team6443.lib.autonomous.ChoreoPather;

import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.config.subsystems.elevator.simulation.SimulatedElevatorConfiguration;

import com.team6443.lib.subsystems.vision.io.limelight.Limelight4HardwareIO;
import com.team6443.lib.subsystems.vision.io.limelight.Limelight4SimIO;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** 
 * High level subsystem factories intended to be called from RobotContainer when the robot Subsystem representation is being constructed
 */
public class SubsystemFactory {

    // Variable to allow one point of updating the field year 
    private static final Field2025 kCurrentYear = Field2025.getInstance();

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
                
                elevator = new ElevatorSubsystem(
                    elevatorConfig,
                    new ElevatorSimIO(
                        elevatorConfig, 
                        simulatedElevatorConfig
                    )
                );
                break;

            // ---- Physical instance of elevator ----
            case REPLAY: // fall down to default
            case REAL:
            default:
                elevator = new ElevatorSubsystem(
                    elevatorConfig,
                    new ElevatorHardwareIO(elevatorConfig)
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
                        new DrivetrainSimIO(
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
                    new DrivetrainHardwareIO(
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
                    new Limelight4SimIO(
                        RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(0),
                        kCurrentYear,
                        () -> SimulatedRobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed(),
                        (cameraSim, cameraTransform) -> { SimulatedRobotState.get().addCameraToVisionSimulation(cameraSim, cameraTransform); }
                    ),
                    new Limelight4SimIO(
                        RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(1),
                        kCurrentYear,
                        () -> SimulatedRobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed(),
                        (cameraSim, cameraTransform) -> { SimulatedRobotState.get().addCameraToVisionSimulation(cameraSim, cameraTransform); }
                    ),
                    new Limelight4SimIO(RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(2),
                        kCurrentYear,
                        () -> SimulatedRobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed(),
                        (cameraSim, cameraTransform) -> { SimulatedRobotState.get().addCameraToVisionSimulation(cameraSim, cameraTransform); }
                    ),
                    new Limelight4SimIO(RobotRuntimeConstants.kRobotConfiguration.getSimulatedCameraConfigurations().get(3),
                        kCurrentYear,
                        () -> SimulatedRobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed(),
                        (cameraSim, cameraTransform) -> { SimulatedRobotState.get().addCameraToVisionSimulation(cameraSim, cameraTransform); }
                    )
                );
                

            // ---- Physical instance of drivetrain ----
            case REPLAY: // fall down to default
            case REAL:
            default:
                return new VisionSubsystem(
                    new Limelight4HardwareIO(
                        RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(0), 
                        kCurrentYear, 
                        () -> RobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed()
                    ),
                    new Limelight4HardwareIO(
                        RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(1), 
                        kCurrentYear,
                        () -> RobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed()
                    ),
                    new Limelight4HardwareIO(
                        RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(2), 
                        kCurrentYear,
                        () -> RobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed()
                    ),
                    new Limelight4HardwareIO(
                        RobotRuntimeConstants.kRobotConfiguration.getCameraConfigurations().get(3), 
                        kCurrentYear,
                        () -> RobotState.get().getLatestFieldRobotPose(),
                        () -> RobotState.get().getLatestDesiredFieldRelativeChassisSpeed()
                    )
                );
        }
    }

    public static ChoreoPather createChoreoPather(DrivetrainSubsystem drivetrainSubsystem){
        ChoreoPather pathing = 
        new ChoreoPather(RobotRuntimeConstants.kRobotConfiguration.getChoreoPatherConfiguration())
            .withAutoFactory(
                () -> RobotState.get().getLatestFieldRobotPose(),
                drivetrainSubsystem::resetOdometry,
                drivetrainSubsystem::setControl,
                true,
                drivetrainSubsystem
            );  

        drivetrainSubsystem.setChoreoPatherLoggable(pathing);
        return pathing;
    }
}
