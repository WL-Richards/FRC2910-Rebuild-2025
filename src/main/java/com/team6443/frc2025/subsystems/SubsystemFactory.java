// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.frc2025.subsystems.elevator.ElevatorSubsystem;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.factories.motors.TalonFXFactory;
import com.team6443.lib.motors.hardware.TalonFXIO;
import com.team6443.lib.subsystems.simulation.SimulatedElevator;
import com.team6443.lib.subsystems.simulation.SimulatedElevator.SimulatedElevatorConfiguration;

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
    
}
