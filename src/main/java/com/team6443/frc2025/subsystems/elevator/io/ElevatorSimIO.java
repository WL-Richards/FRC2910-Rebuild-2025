// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.elevator.io;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.config.subsystems.elevator.simulation.SimulatedElevatorConfiguration;
import com.team6443.lib.core.motors.interfaces.MotorIO;
import com.team6443.lib.subsystems.elevator.simulation.SimulatedElevator;


/** Elevator IO implementation for simulation */
public class ElevatorSimIO extends ElevatorHardwareIO {
    private final SimulatedElevator simElevator;
    
    public ElevatorSimIO(
        ServoMotorFollowerConfiguration<TalonFXConfiguration> elevatorConfig,
        SimulatedElevatorConfiguration simulatedElevatorConfig
    ){
        super(elevatorConfig);
        simElevator = new SimulatedElevator(elevatorConfig, simulatedElevatorConfig);
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        simElevator.updateLog(standardPrefix, inputPrefix);
    }

    @Override
    public MotorIO getLeadMotor() {
        return simElevator.getLeadTalon();
    }

    @Override
    public MotorIO[] getFollowerMotors() {
        return simElevator.getFollowerTalons();
    }

}
