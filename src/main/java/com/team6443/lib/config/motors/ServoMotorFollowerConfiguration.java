// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.motors;

import java.util.List;

import com.team6443.lib.motors.interfaces.MotorIO.FollowDirection;

/**
 * Represents the configuration for a servo motor that has one or more follower motors.
 * <p>
 * This class defines how follower motors are set up relative to a primary servo motor.
 * Each follower motor can have its own configuration and may be set to follow
 * in the same or inverted direction as the leader.
 *
 * @param <C> The type of the configuration object used for each motor.
 */
public class ServoMotorFollowerConfiguration<C> extends ServoMotorConfiguration<C>{

    /**
     * Defines how an individual follower motor is configured relative to a leader motor.
     * <p>
     * Each follower motor includes its own configuration and an inversion flag
     * that determines whether it should follow the leader in the same or opposite direction.
     *
     * @param <C> The type of motor configuration object associated with the follower motor.
     */
    public static class FollowerConfiguration<C> {

        /**
         * Indicates how this motor's direction should be following the leader
         */
        public FollowDirection followDirection = FollowDirection.SAME;

        /**
         * Configuration of the follower motor, containing its setup details
         * such as motor ID, PID constants, and other parameters.
         */
        public ServoMotorConfiguration<C> config = null;

        public FollowerConfiguration(ServoMotorConfiguration<C> config){
            this.config = config;
        }

        /**
         * Sets the configuration for this follower motor.
         *
         * @param config The {@link ServoMotorConfiguration} object containing the follower's settings.
         * @return This {@link FollowerConfiguration} instance, for chaining.
         */
        public FollowerConfiguration<C> withConfig(ServoMotorConfiguration<C> config) {
            this.config = config;
            return this;
        }

        /**
         * Sets the follow direction for this follower motor.
         *
         * @param direction The {@link FollowDirection} indicating whether the follower
         *                  should mirror or invert the leader's movement.
         * @return This {@link FollowerConfiguration} instance, for chaining.
         */
        public FollowerConfiguration<C> withFollowDirection(FollowDirection direction) {
            this.followDirection = direction;
            return this;
        }
    }

    /**
     * The list of follower motors that should follow the leader motor.
     */
    public List<FollowerConfiguration<C>> followerConfigurations;

    public ServoMotorFollowerConfiguration(C config){
        super.withConfig(config);
    }

    public ServoMotorFollowerConfiguration(){}

    /**
     * Sets the follower motor configurations for the follower servo motors.
     *
     * @param motors An array of {@link FollowerConfiguration} objects defining each follower motor.
     * @return This {@link ServoMotorFollowerConfiguration} instance, for chaining.
     */
    public ServoMotorFollowerConfiguration<C> withFollowerConfigs(List<FollowerConfiguration<C>> configs) {
        this.followerConfigurations = configs;
        return this;
    }
    
}
