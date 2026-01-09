// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.elevator.simulation;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.team6443.lib.config.motors.MotorFollowerConfiguration;
import com.team6443.lib.config.subsystems.elevator.simulation.SimulatedElevatorConfiguration;
import com.team6443.lib.core.logging.Loggable;
import com.team6443.lib.core.motors.interfaces.MotorIO.FollowDirection;
import com.team6443.lib.core.motors.io.TalonFXHardwareIO;
import com.team6443.lib.core.motors.io.TalonFXSimIO;
import com.team6443.lib.subsystems.elevator.visualizations.ElevatorVizualizer;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

/** 
 * Handles running an elevator simulation on the given hardware 
 */
public class SimulatedElevator implements Loggable{

    /**
     * Tracks the current state of the simulated elevator
     */
    class SimulatedElevatorInputs {
        double SupplyVoltage;
        double SimVoltage;
        double SimPositionM;
        double RotorPosition;
        double RotorVel;
    }

    protected ElevatorVizualizer elevatorViz = new ElevatorVizualizer();
    protected SimulatedElevatorInputs inputs = new SimulatedElevatorInputs();

    // Setup our instances of our simulated talons and elevator configs
    protected MotorFollowerConfiguration<TalonFXConfiguration> config;
    protected SimulatedElevatorConfiguration elevatorSimulationConfiguration;

    protected TalonFXSimIO leadTalonSimulation;
    protected TalonFXSimIO[] followerTalonSimulations;

    // Elevator simulation itself + update info
    protected ElevatorSim elevatorSimulation;
    protected Notifier simNotifier = null;
    protected double lastUpdateTimestamp = 0.0;

    public SimulatedElevator(MotorFollowerConfiguration<TalonFXConfiguration> config, SimulatedElevatorConfiguration simConfig){
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
        leadTalonSimulation = new TalonFXSimIO(config);

        // Create instances of the simulated elevator talon for all the followers
        followerTalonSimulations = new TalonFXSimIO[config.followerConfigurations.size()];
        for(int i = 0; i < config.followerConfigurations.size(); i++){
            followerTalonSimulations[i] = new TalonFXSimIO(
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

    @Override
    public void updateLog(String standardPrefix, String inputPrefix){
        Logger.recordOutput(standardPrefix + "/Simulation/SupplyVoltage", inputs.SupplyVoltage);
        Logger.recordOutput(standardPrefix + "/Simulation/Voltage", inputs.SimVoltage);
        Logger.recordOutput(standardPrefix + "/Simulation/PositionMeters", inputs.SimPositionM);
        Logger.recordOutput(standardPrefix + "/Simulation/RotorPosition", inputs.RotorPosition);
        Logger.recordOutput(standardPrefix + "/Simulation/VelocityMS", elevatorSimulation.getVelocityMetersPerSecond());

        elevatorViz.updateViz(inputs.SimPositionM);
        elevatorViz.updateLog(standardPrefix, inputPrefix);
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

        inputs.SupplyVoltage = RobotController.getBatteryVoltage();
        simState.setSupplyVoltage(inputs.SupplyVoltage);

        inputs.SimVoltage = applyFriction(simState.getMotorVoltage(), elevatorSimulationConfiguration.frictionVoltage);
        elevatorSimulation.setInput(inputs.SimVoltage);

        double timestamp = Timer.getFPGATimestamp();
        elevatorSimulation.update(timestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = timestamp;

        // Find current state of sim in M
        inputs.SimPositionM = elevatorSimulation.getPositionMeters();

        // Mutate rotor position
        inputs.RotorPosition = inputs.SimPositionM / elevatorSimulationConfiguration.meterToRotorRatio;
        simState.setRawRotorPosition(inputs.RotorPosition);

        // Mutate rotor vel
        inputs.RotorVel = elevatorSimulation.getVelocityMetersPerSecond() / elevatorSimulationConfiguration.meterToRotorRatio;
        simState.setRotorVelocity(inputs.RotorVel);
        

        for (int i = 0; i < followerTalonSimulations.length; ++i) {
            followerTalonSimulations[i].getSimState().setRawRotorPosition(
                inputs.RotorPosition * (this.config.followerConfigurations.get(i).followDirection == FollowDirection.INVERT ? 1.0 : -1.0));
                    
            followerTalonSimulations[i].getSimState().setRotorVelocity(
                inputs.RotorVel * (this.config.followerConfigurations.get(i).followDirection == FollowDirection.INVERT ? 1.0 : -1.0));
        }

        
    }

    public TalonFXHardwareIO getLeadTalon() {
        return leadTalonSimulation;
    }

    public TalonFXHardwareIO[] getFollowerTalons() {
        return followerTalonSimulations;
    }
}
