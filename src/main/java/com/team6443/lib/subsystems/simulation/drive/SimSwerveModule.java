// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.simulation.drive;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;


import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

/** 
 * Represents the simulation occurring for a single swerve module within the system
*/
class SimSwerveModule {
    public final SwerveModuleConstants<?, ?, ?> moduleConstants;
    public final SwerveModuleSimulation moduleSimulation;       // Maple sim swerve module simulation component

    public SimSwerveModule(
        SwerveModuleConstants<?, ?, ?> moduleConstants,
        SwerveModuleSimulation moduleSimulation,
        SwerveModule<TalonFX,TalonFX,CANcoder> module){
            this.moduleConstants = moduleConstants;

            // Setup module simulation
            this.moduleSimulation = moduleSimulation;
            this.moduleSimulation.useDriveMotorController(
                new TalonFXMotorControllerSim(module.getDriveMotor())
            );

            this.moduleSimulation.useSteerMotorController(
                new TalonFXMotorControllerWithCANcoderSim(
                    module.getSteerMotor(),
                    module.getEncoder()
                )
            );
            
    }
}
