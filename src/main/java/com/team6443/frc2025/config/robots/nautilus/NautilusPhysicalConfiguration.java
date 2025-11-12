// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.frc2025.config.robots.nautilus;

import edu.wpi.first.math.util.Units;

/**
 * Represents the physical state of the nautilus robot
 */
public class NautilusPhysicalConfiguration {

    // --- Physical Properties --- 
    public final double kWheelBaseLengthM = Units.inchesToMeters(22.75);
    public final double kWheelTrackWidthM = Units.inchesToMeters(22.75);

    public final double kBumperLengthM = Units.inchesToMeters(35.625);
    public final double kBumperWidthM = Units.inchesToMeters(35.625);

    public final double kRobotWeightPounds = 150;
}
