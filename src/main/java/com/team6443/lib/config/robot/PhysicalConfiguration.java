// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.robot;

import edu.wpi.first.math.util.Units;

/** 
 * Defines the critical physical dimensions and properties of the robot
 * used for kinematics, odometry, and simulation.
 *  - Wheel Base and Track Width (M)
 *  - Bumper Width and Bumper Length (M)
 *  - Robot Weight (Pounds)
 * 
 */
public class PhysicalConfiguration {

    // --- Physical Dimensions (Meters) ---

    /**
     * The distance (in meters) between the center of the front and rear axles
     * of the drivetrain. This is the 'X' dimension in a swerve chassis.
     */
    public double kWheelBaseLengthMeters = Units.inchesToMeters(22.75);

    /**
     * The distance (in meters) between the center of the left and right wheels
     * on the same axle. This is the 'Y' dimension in a swerve chassis.
     */
    public double kWheelTrackWidthMeters = Units.inchesToMeters(22.75);

    /**
     * The total length of the robot, including bumpers (in meters), along the X-axis (forward/back).
     * Used primarily for geometry and simulation bounding boxes.
     */
    public double kBumperLengthMeters = Units.inchesToMeters(35.625);

    /**
     * The total width of the robot, including bumpers (in meters), along the Y-axis (left/right).
     * Used primarily for geometry and simulation bounding boxes.
     */
    public double kBumperWidthMeters = Units.inchesToMeters(35.625);

    // --- Physical Properties (Weight & Friction) ---

    /**
     * The total weight of the robot, including all components, battery, and bumpers (in pounds).
     * This value is essential for accurate physics simulation (e.g., in MapleSim/WPILib).
     */
    public double kRobotWeightPounds = 150;

    /**
     * The coefficient of friction (CoF) between the wheel material and the carpet.
     * Higher values (typically > 1.0) indicate better grip; used in simulation to model slip limits.
     */
    public double kWheelCoefficientOfFriction = 1.2;

    /**
     * Set the wheelbase length in meters.
     * @param lengthMeters The length of the wheelbase in meters.
     * @return This configuration object for chaining.
     */
    public PhysicalConfiguration withWheelBaseLengthM(double lengthMeters) {
        this.kWheelBaseLengthMeters = lengthMeters;
        return this;
    }

    /**
     * Set the wheel track width in meters.
     * @param widthMeters The width of the wheel track in meters.
     * @return This configuration object for chaining.
     */
    public PhysicalConfiguration withWheelTrackWidthM(double widthMeters) {
        this.kWheelTrackWidthMeters = widthMeters;
        return this;
    }

    /**
     * Set the bumper length in meters.
     * @param lengthMeters The length of the bumpers in meters.
     * @return This configuration object for chaining.
     */
    public PhysicalConfiguration withBumperLengthM(double lengthMeters) {
        this.kBumperLengthMeters = lengthMeters;
        return this;
    }

    /**
     * Set the bumper width in meters.
     * @param widthMeters The width of the bumpers in meters.
     * @return This configuration object for chaining.
     */
    public PhysicalConfiguration withBumperWidthM(double widthMeters) {
        this.kBumperWidthMeters = widthMeters;
        return this;
    }

    /**
     * Set the robot weight in pounds.
     * @param weightPounds The weight of the robot in lbs.
     * @return This configuration object for chaining.
     */
    public PhysicalConfiguration withRobotWeightPounds(double weightPounds) {
        this.kRobotWeightPounds = weightPounds;
        return this;
    }

    /**
     * Set the coefficient of friction for the wheels.
     * @param cof The coefficient of friction.
     * @return This configuration object for chaining.
     */
    public PhysicalConfiguration withWheelCoefficientOfFriction(double cof) {
        this.kWheelCoefficientOfFriction = cof;
        return this;
    }
}
