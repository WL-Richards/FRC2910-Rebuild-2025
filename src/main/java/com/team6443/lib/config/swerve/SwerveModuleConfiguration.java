// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.swerve;

import com.ctre.phoenix6.configs.ParentConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.wrappers.ConfigureSlot0Gains;
import com.team6443.lib.mechanics.MultistageGearBox;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * Base configuration class used to describe a 
 */
public abstract class SwerveModuleConfiguration<DC extends ParentConfiguration, SC extends ParentConfiguration, SE extends ParentConfiguration> {

    protected final String moduleName;
    protected final CANDeviceID driveMotorID;
    protected final CANDeviceID steerMotorID;
    protected final CANDeviceID steerEncoderID;

    // Physical properties 
    protected double kWheelRadiusM = 0;

    // Friction voltages
    protected double kDriveFrictionVoltage = 0; // The minimum voltage required for the drive motor to begin moving
    protected double kSteerFrictionVoltage = 0; // The minimum voltage required for the steer motor to begin moving

    // Rotational inertia
    protected double kDriveInertia = 0; // The rotational inertia in the drive system (kg * m^2) and represent rotational resistance to acceleration
    protected double kSteerInertia = 0; // The rotational inertia in the drive system (kg * m^2) and represent rotational resistance to acceleration

    // Motor Gains configured for the drive swerve motors
    protected ConfigureSlot0Gains kDriveMotorGains = null;

    // Motor Gains configured for the steer swerve motors
    protected ConfigureSlot0Gains kSteerMotorGains = null;

    // Swerve module encoder offsets
    // Offset from what the encoder thinks is 0 to the true zero of the module in ROTATIONS
    protected double kSteerEncoderOffsetRotations = 0;

    // Motor Configurations

    // Drive Motor
    protected double kDriveMotorSupplyCurrentLimit = 0; // The amount of current (amps) that this motor is allowed to pull from the battery, if exceeded voltage will be reduced to avoid brownouts
    protected double kDriveMotorStatorCurrentLimit = 0;  // The amount of current (amps) that motor is allowed to draw up to
    protected double kDriveMotorSlipCurrent = 0; // The amount of current (amps) that can be applied to the drive wheel before it slips (120 basically means it doesn't slip)
    protected DC kDriveMotorConfiguration = null;

    // Steer Motor
    protected double kSteerMotorSupplyCurrentLimit = 0; // amps
    protected double kSteerMotorStatorCurrentLimit = 0;  // amps
    protected SC kSteerMotorConfiguration = null;

    // Steer Encoder
    protected SE kSteerEncoderConfiguration = null;

    // Gearing
    // Gearbox between the drive motor and drive wheel of this swerve module
    protected MultistageGearBox kDriveGearBox = null;

    // Gearbox between the steer motor and the drive wheel of this swerve module
    protected MultistageGearBox kSteerGearBox = null;

    // CAN
    protected CANDeviceID kDriveMotor = null;
    protected CANDeviceID kSteerMotor = null;
    protected CANDeviceID kSteerEncoder = null;

    // Position
    protected Translation2d kLocationOffset = null;

    // Theoretical robot values
    protected double kMaxRobotSpeedMeterPerSecond = -1;

    // Motor properties
    protected DCMotor kDriveMotorType = null;

    // Swerve constants
    protected SwerveModuleConstants<DC, SC, SE> kModuleConstants = null;

    public SwerveModuleConfiguration(String moduleName, CANDeviceID driveMotorID, CANDeviceID steerMotorID, CANDeviceID steerEncoderID){
        this.moduleName = moduleName;
        this.driveMotorID = driveMotorID;
        this.steerMotorID = steerMotorID;
        this.steerEncoderID = steerEncoderID;
    };

    /**
     * Set the minimum voltage required to move the swerve drive wheel
     * @param voltage Voltage to be used achieve movement
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveFrictionVoltage(double voltage){
        this.kDriveFrictionVoltage = voltage;
        return this;
    }

    /**
     * Set the minimum voltage required to turn the swerve steer wheel
     * @param voltage Steer voltage to be used
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withSteerFrictionVoltage(double voltage){
        this.kSteerFrictionVoltage = voltage;
        return this;
    }

     /**
     * Set the rotational inertia for the drive system.
     * Represents the rotational resistance to acceleration (in kg·m²).
     * @param inertia Rotational inertia for the drive system (kg·m²)
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveInertia(double inertia){
        this.kDriveInertia = inertia;
        return this;
    }

    /**
     * Set the rotational inertia for the steer system.
     * Represents the rotational resistance to acceleration (in kg·m²).
     * @param inertia Rotational inertia for the steer system (kg·m²)
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withSteerInertia(double inertia){
        this.kSteerInertia = inertia;
        return this;
    }

    /**
     * Set the PID and feedforward gains for the drive motor's Slot0 controller.
     *
     * @param kP Proportional gain — scales the response based on position error
     * @param kI Integral gain — accumulates steady-state error over time
     * @param kD Derivative gain — predicts and counteracts rate of change in error
     * @param kV Velocity feedforward gain — compensates for expected velocity demand
     * @param kS Static friction feedforward gain — compensates for motor stiction (voltage needed to start motion)
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorGains(
        double kP,
        double kI,
        double kD,
        double kV,
        double kS
    ){
        this.kDriveMotorGains = new ConfigureSlot0Gains(kP, kI, kD, kV, kS);
        return this;
    }

    /**
     * Set the PID and feedforward gains for the drive motor's Slot0 controller.
     *
     * @param slot The configured slot 0 gains
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorGains(
        ConfigureSlot0Gains slot
    ){
        this.kDriveMotorGains = slot;
        return this;
    }

    /**
     * Set the PID and feedforward gains for the drive motor's Slot0 controller.
     *
     * @param slot The configured slot 0 gains
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withWheelRadiusM(double radiusM){
        this.kWheelRadiusM = radiusM;
        return this;
    }

    /**
     * Set the PID/FF gains for the steer motor's Slot0 controller.
     * @param kP Proportional gain — scales the response based on position error
     * @param kI Integral gain — accumulates steady-state error over time
     * @param kD Derivative gain — predicts and counteracts rate of change in error
     * @param kV Velocity feedforward gain — compensates for expected velocity demand
     * @param kS Static friction feedforward gain — compensates for motor stiction (voltage needed to start motion)
     */
    public SwerveModuleConfiguration<DC, SC, SE> withSteerMotorGains(double kP,
        double kI,
        double kD,
        double kV,
        double kS
    ){
        this.kSteerMotorGains = new ConfigureSlot0Gains(kP, kI, kD, kV, kS);
        return this;
    }

     /**
     * Set the PID and feedforward gains for the steer motor's Slot0 controller.
     *
     * @param slot The configured slot 0 gains
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withSteerMotorGains(
        ConfigureSlot0Gains slot
    ){
        this.kSteerMotorGains = slot;
        return this;
    }

     /**
     * Set the absolute encoder offset for the module (in rotations).
     * Offset = (true zero) - (reported zero), in ROTATIONS.
     * @param rotations Offset in rotations
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withEncoderOffsetRotations(double rotations){
        this.kSteerEncoderOffsetRotations = rotations;
        return this;
    }

    /**
     * Set the supply current limit for the drive motor, in amps.
     * If exceeded, voltage may be reduced to prevent brownouts.
     * @param amps Maximum allowed supply current (A)
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorSupplyCurrentLimit(double amps){
        this.kDriveMotorSupplyCurrentLimit = amps;
        return this;
    }

    /**
     * Set the stator (motor) current limit for the drive motor, in amps.
     * This is the maximum motor current draw the controller will allow.
     * @param amps Maximum allowed stator current (A)
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorStatorCurrentLimit(double amps){
        this.kDriveMotorStatorCurrentLimit = amps;
        return this;
    }

    /**
     * Set the slip current threshold for the drive wheel, in amps.
     * Above this current the wheel is considered likely to slip.
     * @param amps Slip current threshold (A)
     * @return The SwerveModuleConfiguration being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorSlipCurrent(double amps){
        this.kDriveMotorSlipCurrent = amps;
        return this;
    }

    /**
     * Set the gearbox configuration between the drive motor and the drive wheel.
     * 
     * The drive gearbox defines the total gear reduction between the motor output
     * and the wheel rotation, and directly affects the relationship between
     * motor speed, torque, and wheel linear velocity.
     *
     * @param gearBox The {@link MultistageGearBox} instance representing the drive gearbox
     * @return The {@link SwerveModuleConfiguration} being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveGearBox(MultistageGearBox gearBox) {
        this.kDriveGearBox = gearBox;
        return this;
    }

    /**
     * Set the gearbox configuration between the steer motor and the steering output.
     * 
     * The steer gearbox defines the total reduction between the motor and the steering mechanism,
     * determining how motor rotations translate to wheel angle changes. 
     * A higher ratio provides finer control but slower steering response.
     *
     * @param gearBox The {@link MultistageGearBox} instance representing the steer gearbox
     * @return The {@link SwerveModuleConfiguration} being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withSteerGearBox(MultistageGearBox gearBox) {
        this.kSteerGearBox = gearBox;
        return this;
    }

    /**
     * Set the CAN device ID for the drive motor controller.
     * 
     * This identifies which CAN bus and device ID correspond to the drive motor.
     *
     * @param canDevice The {@link CANDeviceID} representing the drive motor
     * @return The {@link SwerveModuleConfiguration} being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveMotor(CANDeviceID canDevice) {
        this.kDriveMotor = canDevice;
        return this;
    }

    /**
     * Set the CAN device ID for the steer motor controller.
     * 
     * This identifies which CAN bus and device ID correspond to the steer motor.
     *
     * @param canDevice The {@link CANDeviceID} representing the steer motor
     * @return The {@link SwerveModuleConfiguration} being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withSteerMotor(CANDeviceID canDevice) {
        this.kSteerMotor = canDevice;
        return this;
    }

    /**
     * Set the CAN device ID for the steer encoder.
     * 
     * The steer encoder provides the absolute angular position of the swerve module,
     * used for precise steering control and module zeroing.
     *
     * @param canDevice The {@link CANDeviceID} representing the steer encoder
     * @return The {@link SwerveModuleConfiguration} being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withSteerEncoder(CANDeviceID canDevice) {
        this.kSteerEncoder = canDevice;
        return this;
    }

    /**
     * Set the physical location offset of this swerve module relative to the robot center.
     * 
     * This represents the position of the module on the robot frame, measured in meters
     * along the X and Y axes from the robot's center. 
     * Positive X is forward, and positive Y is to the left when viewed from above.
     * 
     * This value is used in kinematic and odometry calculations to properly determine
     * wheel velocity vectors, rotation about the robot center, and field-relative motion.
     *
     * @param x The X offset from the robot center in meters (forward is positive)
     * @param y The Y offset from the robot center in meters (left is positive)
     * @return The {@link SwerveModuleConfiguration} being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withLocationOffset(double x, double y) {
        this.kLocationOffset = new Translation2d(x,y);
        return this;
    }

    /**
     * Set the motor type used for the drive motor of this swerve module.
     * 
     * This specifies the {@link DCMotor} model that represents the physical characteristics 
     * of the drive motor, including nominal voltage, free speed, stall torque, and current draw.
     * The motor type is typically used in feedforward calculations, simulation models,
     * and to characterize drivetrain performance.
     * 
     * Common examples include:
     * <ul>
     *   <li>{@code DCMotor.getFalcon500(1)}</li>
     *   <li>{@code DCMotor.getNEO(1)}</li>
     *   <li>{@code DCMotor.getKrakenX60(1)}</li>
     * </ul>
     *
     * @param motor The {@link DCMotor} instance representing the drive motor type
     * @return The {@link SwerveModuleConfiguration} being configured
     */
    public SwerveModuleConfiguration<DC, SC, SE> withDriveMotorType(DCMotor motor) {
        this.kDriveMotorType = motor;
        return this;
    }

    /**
     * Retrieve the theoretical maximum linear speed of this swerve module in meters per second.
     * 
     * This value is typically derived from the free speed of the drive motor, the total drive 
     * gear reduction, and the wheel diameter:
     * <pre>
     *     maxSpeed = (motorFreeSpeedRPM / gearRatio) * (wheelCircumferenceMeters / 60)
     * </pre>
     * 
     * It represents the fastest linear velocity the module could achieve under ideal
     * (no-load) conditions. In practice, real-world values will be lower due to 
     * drivetrain inefficiencies, voltage drops, and traction limitations.
     *
     * @return The maximum attainable module speed in meters per second
     */
    public double getMaxSpeedMetersPerSecond(){
        if(kMaxRobotSpeedMeterPerSecond == -1){
            kMaxRobotSpeedMeterPerSecond = (Units.radiansPerSecondToRotationsPerMinute(kDriveMotorType.freeSpeedRadPerSec) / 60.0) *
                                            Math.PI *
                                            kWheelRadiusM / kDriveGearBox.getTotalRatio();
        }
        return  kMaxRobotSpeedMeterPerSecond;
    }

    /**
     * Retrieve the complete set of constants configured for this swerve module.
     * This includes drive, steer, and encoder parameters specific to the module.
     *
     * @return A {@link SwerveModuleConstants} instance containing all configuration data
     */
    public abstract SwerveModuleConstants<DC, SC, SE> getModuleConstants();

    /**
     * Retrieve the configuration object for the drive motor associated with this swerve module.
     * Contains all TalonFX or motor controller settings such as PID gains, limits, and inversion.
     *
     * @return The drive motor configuration object of type {@code DC}
     */
    public abstract DC getDriveMotorConfiguration();

    /**
     * Retrieve the configuration object for the steer motor associated with this swerve module.
     * Contains all TalonFX or motor controller settings such as PID gains, limits, and inversion.
     *
     * @return The steer motor configuration object of type {@code SC}
     */
    public abstract SC getSteerMotorConfiguration();

    /**
     * Retrieve the configuration object for the steer encoder associated with this swerve module.
     * Contains all sensor-related settings such as offsets, direction, and conversion factors.
     *
     * @return The steer encoder configuration object of type {@code SE}
     */
    public abstract SE getSteerEncoderConfiguration();

    

}
