// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.factories.motors.TalonFXFactory;
import com.team6443.lib.motors.hardware.TalonFXIO;
import com.team6443.lib.motors.interfaces.MotorIO;
import com.team6443.lib.subsystems.elevator.ElevatorIO;

/** Hardware IO implementation for an elevator */
public class ElevatorIOHardware implements ElevatorIO{
    private final TalonFXIO leadMotor;
    private final TalonFXIO[] followerMotors;

    public ElevatorIOHardware(
       ServoMotorFollowerConfiguration<TalonFXConfiguration> elevatorConfig
    ){
        // Setup our leader motor based on the configuration 
        leadMotor = TalonFXFactory.createIO(elevatorConfig);

        // Setup our follower motors based on the configuration
        followerMotors = new TalonFXIO[] {
            TalonFXFactory.createIO(elevatorConfig.followerConfigurations.get(0).config)
        };
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {}

    @Override
    public MotorIO getLeadMotor() {
        return leadMotor;
    }

    @Override
    public MotorIO[] getFollowerMotors() {
        return followerMotors;
    }

}
