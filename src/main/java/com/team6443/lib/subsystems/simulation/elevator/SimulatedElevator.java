// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.simulation.elevator;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6443.lib.config.motors.ServoMotorConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration;
import com.team6443.lib.config.motors.ServoMotorFollowerConfiguration.FollowerConfiguration;
import com.team6443.lib.motors.hardware.TalonFXIO;
import com.team6443.lib.motors.interfaces.MotorIO.FollowDirection;
import com.team6443.lib.motors.sim.TalonFXSimIO;
import com.team6443.lib.subsystems.simulation.visualizations.ElevatorVizualizer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** 
 * Handles running an elevator simulation on the given hardware 
 */
public class SimulatedElevator {

    public static class SimulatedElevatorConfiguration {
        public double gearing;
        public double carriageMass; // in KG
        public double drumRadius;
        public double frictionVoltage; // Amount of voltage required to over come static friction


        // This is rotor to the height of the elevator.
        // rotor * this number = height of elevator.
        public double meterToRotorRatio;
    }

    protected ElevatorVizualizer elevatorViz = new ElevatorVizualizer();

    // Setup our instances of our simulated talons and elevator configs
    protected ServoMotorFollowerConfiguration<TalonFXConfiguration> config;
    protected SimulatedElevatorConfiguration elevatorSimulationConfiguration;

    protected TalonFXSimIO leadTalonSimulation;
    protected TalonFXSimIO[] followerTalonSimulations;

    // Elevator simulation itself + update info
    protected ElevatorSim elevatorSimulation;
    protected Notifier simNotifier = null;
    protected double lastUpdateTimestamp = 0.0;

    public SimulatedElevator(ServoMotorFollowerConfiguration<TalonFXConfiguration> config, SimulatedElevatorConfiguration simConfig){
        this.config = config;
        this.elevatorSimulationConfiguration = simConfig;
       
        // Setup elevator sim
        this.elevatorSimulation = new ElevatorSim(
            DCMotor.getKrakenX60(config.followerConfigurations.size() + 1), 
            1.0 / simConfig.gearing, 
            simConfig.carriageMass, 
            simConfig.drumRadius, 
            config.kMinPositionUnits,
            config.kMaxPositionUnits,
            true, 
            0.0
        );

        // Setup lead talon
        leadTalonSimulation = new TalonFXSimIO(config.kCANDevice, config);

        // Create instances of the simulated elevator talon for all the followers
        followerTalonSimulations = new TalonFXSimIO[config.followerConfigurations.size()];
        for(int i = 0; i < config.followerConfigurations.size(); i++){
            followerTalonSimulations[i] = new TalonFXSimIO(
                config.followerConfigurations.get(i).config.kCANDevice,
                config.followerConfigurations.get(i).config
            );
        }

        // Setup sim update to update faster so PID gains behave as expected
        simNotifier =
        new Notifier(
                () -> {
                    updateSimState();
                });
        simNotifier.setName("ElevatorSimNotifier");
        simNotifier.startPeriodic(0.005);

    }

    protected double applyFriction(double motorVoltage, double frictionVoltage) {
        if (Math.abs(motorVoltage) < frictionVoltage) {
            motorVoltage = 0.0;
        } else if (motorVoltage > 0.0) {
            motorVoltage -= frictionVoltage;
        } else {
            motorVoltage += frictionVoltage;
        }
        return motorVoltage;
    }

    protected void updateSimState(){
        TalonFXSimState simState = leadTalonSimulation.getSimState();
        double supplyVoltage = RobotController.getBatteryVoltage();

        Logger.recordOutput("Subsystems/" + config.kConfigurationName + "/Sim/SupplyVoltage", supplyVoltage);

        simState.setSupplyVoltage(supplyVoltage);
        double simVoltage = applyFriction(simState.getMotorVoltage(), elevatorSimulationConfiguration.frictionVoltage);
        
        elevatorSimulation.setInput(simVoltage);
        Logger.recordOutput("Subsystems/" + config.kConfigurationName + "/Sim/SimulatorVoltage", simVoltage);

        double timestamp = Timer.getFPGATimestamp();
        elevatorSimulation.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        // Find current state of sim in M
        double simPositionM = elevatorSimulation.getPositionMeters();
        Logger.recordOutput("Subsystems/" + config.kConfigurationName  + "/Sim/SimulatorPositionMeters", simPositionM);

        // Mutate rotor position
        double rotorPosition = simPositionM / elevatorSimulationConfiguration.meterToRotorRatio;
        simState.setRawRotorPosition(rotorPosition);
        Logger.recordOutput("Subsystems/" + config.kConfigurationName + "/Sim/setRawRotorPosition", rotorPosition);

        // Mutate rotor vel
        double rotorVel = elevatorSimulation.getVelocityMetersPerSecond() / elevatorSimulationConfiguration.meterToRotorRatio;
        simState.setRotorVelocity(rotorVel);
        Logger.recordOutput(
                "Subsystems/" + config.kConfigurationName + "/Sim/SimulatorVelocityMS", elevatorSimulation.getVelocityMetersPerSecond());

        for (int i = 0; i < followerTalonSimulations.length; ++i) {
            followerTalonSimulations[i].getSimState().setRawRotorPosition(
                    rotorPosition * (this.config.followerConfigurations.get(i).followDirection == FollowDirection.INVERT ? 1.0 : -1.0));
                    
            followerTalonSimulations[i].getSimState().setRotorVelocity(
                    rotorVel * (this.config.followerConfigurations.get(i).followDirection == FollowDirection.INVERT ? 1.0 : -1.0));
        }

        elevatorViz.updateViz(simPositionM);
    }

    public TalonFXIO getLeadTalon() {
        return leadTalonSimulation;
    }

    public TalonFXIO[] getFollowerTalons() {
        return followerTalonSimulations;
    }
}
