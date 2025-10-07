// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config.base.wrappers;

import com.ctre.phoenix6.configs.Slot0Configs;

/**
 * Helper class to configure the gains for the drive and steer motors.
 */
public class ConfigureSlot0Gains extends Slot0Configs {
    public ConfigureSlot0Gains(double kP, double kI, double kD, double kV, double kS) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
    }
}
