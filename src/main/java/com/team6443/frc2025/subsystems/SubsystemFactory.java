// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.team6443.frc2025.constants.RobotRuntimeConstants;
import com.team6443.lib.config.motors.ServoMotorCANCoderConfiguration;
import com.team6443.lib.factories.encoder.CANCoderFactory;
import com.team6443.lib.factories.motors.TalonFXFactory;

/** 
 * High level subsystem factories intended to be called from RobotContainer when the robot Subsystem representation is being constructed
 */
public class SubsystemFactory {

    /**
     * Create and return the TestSubsystem constructing it based on current state of bot
     * @return The newly created subsystem
     */
    public static TestSubsystem createTestSubsystem(){
        ServoMotorCANCoderConfiguration<TalonFXConfiguration> config = RobotRuntimeConstants.kRobotConfiguration.getTestSubsystemConfiguration();
        TestSubsystem testSubsystem = new TestSubsystem(
            // Create a new ServoMotorCANCoderConfiguration using a TalonFX with a default TalonFX config
            config,              
            TalonFXFactory.createIO(config), 
            CANCoderFactory.createIO(config.canCoderConfig)
        );

        return testSubsystem;
    }
}
