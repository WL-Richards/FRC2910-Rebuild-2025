// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.wrappers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.GravityTypeValue;

/**
 * Helper class to configure the gains for the drive and steer motors.
 * Yoinked: https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/config/ConfigureSlot0Gains.java
 */
public class ConfigureSlot0Gains extends Slot0Configs {
    public ConfigureSlot0Gains(double kP, double kI, double kD, double kG, double kS, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kS = kS;
        this.kG = kG;
        this.kA = kA;
    }

    public ConfigureSlot0Gains withGravityType(GravityTypeValue type){
        this.GravityType = type;
        return this;
    }
}
