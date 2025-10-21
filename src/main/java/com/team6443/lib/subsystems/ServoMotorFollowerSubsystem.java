// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems;

import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.motors.MotorInputs;
import com.team6443.lib.motors.interfaces.MotorIO;

/**
 * A servo motor subsystem that manages a leader motor with one or more follower motors.
 * <p>
 * This class extends {@link ServoMotorSubsystem} and adds support for synchronized
 * follower motors that mirror the leader's movement. The follower motors are automatically
 * configured and updated each cycle based on their individual {@link ServoMotorFollowerConfiguration.FollowerConfiguration}
 * definitions.
 * <p>
 * Typical use case: multi-motor mechanisms such as drivetrains, elevators, or shooters,
 * where motors must maintain identical (or inverted) behavior.
 * 
 * <h2>Example Usage</h2>
 * <pre>{@code
 * // Example showing a leader motor with two followers
 *
 * // Create leader motor configuration
 * ServoMotorFollowerConfiguration<MyMotorConfig> leaderConfig = 
 *         new ServoMotorFollowerConfiguration<>()
 *             .withConfig(new TalonFXConfiguration());
 *
 * // Define follower configurations
 * ServoMotorFollowerConfiguration.FollowerConfiguration<MyMotorConfig> follower =
 *         new ServoMotorFollowerConfiguration.FollowerConfiguration<MyMotorConfig>()
 *             .withConfig(
 *                  new ServoMotorConfiguration<>("FollowerMotor")
 *                      .withConfig(new TalonFXConfiguration())
 *             )
 *             .withFollowDirection(FollowDirection.SAME);
 * 
 * // Define follower configurations to use with leader
 * leaderConfig.withFollowingMotors(new ServoMotorFollowerConfiguration.FollowerConfiguration[] {
 *     follower
 * });
 * 
 * // Instantiate subsystem
 * ServoMotorFollowerSubsystem<MotorInputs, MotorIO, ServoMotorFollowerConfiguration<MyMotorConfig>> subsystem =
 *     new ServoMotorFollowerSubsystem<>(
 *         leaderInputs,
 *         leaderMotor,
 *         new MotorInputs[] { followerInputs },
 *         new MotorIO[] { followerMotor },
 *         leaderConfig
 *     );
 * 
 * // During periodic, the subsystem automatically updates follower telemetry
 * // and mirrors the leader’s motion according to their configuration.
 * </pre>
 *
 * @param <MI> The type of {@link MotorInputs} used to capture sensor/state data for each motor.
 * @param <M>  The type of {@link MotorIO} implementation controlling the motor hardware.
 * @param <C>  The type of {@link ServoMotorFollowerConfiguration} defining motor relationships.
 */
public abstract class ServoMotorFollowerSubsystem<
                MI extends MotorInputs,
                M extends MotorIO,
                C extends ServoMotorFollowerConfiguration<?>
            > extends ServoMotorSubsystem<MI, M, C> {

    /** Configuration object defining leader and follower setup. */
    protected C leaderConfig;

    /** Input/state containers for each follower motor. */
    protected MI[] followerMotorInputs;

    /** Hardware interface objects for each follower motor. */
    protected M[] followerMotors;

    private String logPrefix;

    /**
     * Constructs a new {@code ServoMotorFollowerSubsystem}.
     *
     * @param leaderMotorInputs  The inputs object representing the leader motor’s sensor readings.
     * @param leaderMotor        The {@link MotorIO} implementation controlling the leader motor.
     * @param followerMotorInputs Array of inputs for each follower motor.
     * @param followerMotors      Array of {@link MotorIO} objects for each follower motor.
     * @param config              The configuration object defining follower relationships and parameters.
     *
     * @throws AssertionError if the number of follower inputs does not match the number of follower motors.
     */
    public ServoMotorFollowerSubsystem(
        MI leaderMotorInputs,
        M leaderMotor,
        MI[] followerMotorInputs,
        M[] followerMotors,
        C config
    ) {
        super(leaderMotorInputs, leaderMotor, config);

        // Setup logging information
        this.logPrefix = "Subsystems/" + config.ConfigurationName;

        // Setup configs
        this.leaderConfig = config;

        // Setup follower motors + inputs
        this.followerMotorInputs = followerMotorInputs;
        this.followerMotors = followerMotors;

        // Ensure follower motors and follower motor inputs are same length
        assert this.followerMotorInputs.length == this.followerMotors.length :
                "Length of follower inputs/io not equal";

        // Configure each follower to follow the leader according to their configuration
        for (int i = 0; i <  config.followerConfigurations.size(); i++) {
            MotorIO motor = followerMotors[i];
            motor.follow(leaderConfig.CANDevice, config.followerConfigurations.get(i).followDirection);
        }
    }

    /**
     * Called periodically during the robot’s main loop.
     * <p>
     * Updates each follower motor’s inputs and logs their telemetry using
     * {@link org.littletonrobotics.junction.Logger}.
     */
    @Override
    public void periodic() {
        super.periodic();
        for (int i = 0; i <  config.followerConfigurations.size(); i++) {
            MotorIO motor = followerMotors[i];
            motor.updateInputs(followerMotorInputs[i]);
            Logger.processInputs(logPrefix + "/" + motor.getName(), followerMotorInputs[i]);
        }
    }

    /**
     * Sets the encoder position for both the leader and all follower motors.
     *
     * @param position The new encoder position (in configured units).
     */
    @Override
    protected void setEncoderPosition(double position) {
        super.setEncoderPosition(position);
        for (M follower : followerMotors) {
            follower.setCurrentEncoderPosition(position);
        }
    }

    /**
     * Zeros the encoder position for both the leader and all follower motors.
     */
    @Override
    protected void zeroEncoderPosition() {
        super.zeroEncoderPosition();
        for (M follower : followerMotors) {
            follower.zeroEncoderPosition();
        }
    }

    /**
     * Computes the average current position across the leader and all followers.
     *
     * @return The average position in user-defined units.
     */
    @Override
    public double getCurrentPosition() {
        double averagePosition = motorInputs.unitPosition;
        for (MI followerInput : followerMotorInputs) {
            averagePosition += followerInput.unitPosition;
        }
        return averagePosition / (config.followerConfigurations.size() + 1);
    }

    /**
     * Computes the average current velocity across the leader and all followers.
     *
     * @return The average velocity in user-defined units per second.
     */
    @Override
    public double getCurrentVelocity() {
        double averageVelocity = motorInputs.velocityUnitsPerSecond;
        for (MI followerInput : followerMotorInputs) {
            averageVelocity += followerInput.velocityUnitsPerSecond;
        }
        return averageVelocity / (config.followerConfigurations.size() + 1);
    }

    /**
     * Helper function to generate a same sized list to motors of motor inputs
     * @param motors List of motors the inputs are being generated for
     * @return Array of motor inputs that were generated to match
     */
    protected static MotorInputs[] generateDefaultFollowerInputs(MotorIO[] motors){
        MotorInputs[] inputs = new MotorInputs[motors.length];
        for(int i = 0; i < motors.length; i++){
            inputs[i] = new MotorInputs();
        }
        return inputs;
    }
}
