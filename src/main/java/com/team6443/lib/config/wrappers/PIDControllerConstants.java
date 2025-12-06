// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.wrappers;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

/** * Immutable data class for storing PID Controller constants (P, I, D, F, IZone, and output limits).
 * This class uses the PIDControllerConstants pattern to facilitate easy and readable initialization.
 */
public class PIDControllerConstants {
    public Double kP = null;
    public Double kI = null;
    public Double kD = null;
    public Double kIZone = null; // Integral zone limit
    public Pair<Double, Double> kContinuousInput = null; // Will be non-null if continuous input is enabled
    public Double kTolerance = null; // Output Tolerance


    /**
     * Sets the Proportional Gain (kP).
     * @param kP The proportional gain value.
     * @return The PIDControllerConstants instance for chaining.
     */
    public PIDControllerConstants withP(double kP) {
        this.kP = kP;
        return this;
    }

    /**
     * Sets the Integral Gain (kI).
     * @param kI The integral gain value.
     * @return The PIDControllerConstants instance for chaining.
     */
    public PIDControllerConstants withI(double kI) {
        this.kI = kI;
        return this;
    }

    /**
     * Sets the Derivative Gain (kD).
     * @param kD The derivative gain value.
     * @return The PIDControllerConstants instance for chaining.
     */
    public PIDControllerConstants withD(double kD) {
        this.kD = kD;
        return this;
    }

    /**
     * Tolerance to give the PID Controller
     * @param tolerance How much tolerance to grant the controller.
     * @return The PIDControllerConstants instance for chaining.
     */
    public PIDControllerConstants withTolerance(double tolerance) {
        this.kTolerance = tolerance;
        return this;
    }

    /**
     * Sets the Integral Zone (IZone or I-Range).
     * @param iZone The integral zone value.
     * @return The PIDControllerConstants instance for chaining.
     */
    public PIDControllerConstants withIZone(double iZone) {
        this.kIZone = iZone;
        return this;
    }


    /**
     * Sets the continuous input to the desired range
     * @param lowerBound Lower bound of the continuous input
     * @param upperBound Upper bound of the continuous input
     * @return The PIDControllerConstants instance for chaining.
     */
    public PIDControllerConstants withContinuousInput(double lowerBound, double upperBound) {
        this.kContinuousInput = new Pair<Double,Double>(lowerBound, upperBound);
        return this;
    }

    /**
     * Generate a new PID controller from the given controller constants
     * @return A newly created PID controller
     */
    public PIDController generateController(){
        if(kP != null && kI != null && kD != null){
            PIDController controller = new PIDController(kP, kI, kD);

            // Set I-zone if supplied
            if(kIZone != null){
                controller.setIZone(kIZone);
            }

            // Set tolerance if supplied
            if(kTolerance != null){
                controller.setTolerance(kTolerance);
            }

            // Enable continuous input if supplied
            if(kContinuousInput != null){
                controller.enableContinuousInput(kContinuousInput.getFirst(), kContinuousInput.getSecond());
            }

            return controller;
        }
        else{
            DriverStation.reportError("Attempted to generate PID controller where P, I, or D gains were unset", true);
            return null;
        }
    }
}