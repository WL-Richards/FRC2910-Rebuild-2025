package com.team6443.lib.config.subsystems.elevator.simulation;

public class SimulatedElevatorConfiguration {
    public double gearing;
    public double carriageMass; // in KG
    public double drumRadius;
    public double frictionVoltage; // Amount of voltage required to over come static friction


    // This is rotor to the height of the elevator.
    // rotor * this number = height of elevator.
    public double meterToRotorRatio;
}
