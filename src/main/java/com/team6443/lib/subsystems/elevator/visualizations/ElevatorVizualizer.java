// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems.elevator.visualizations;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.team6443.lib.core.logging.Loggable;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * Mechnisim 2D visiaulizer for an elevator subsystem
 */
public class ElevatorVizualizer implements Loggable{
    private Color8Bit white = new Color8Bit(Color.kWhite);
    private LoggedMechanism2d viz2d = new LoggedMechanism2d(1.27, 2.032);
    private final LoggedMechanismRoot2d root = viz2d.getRoot("elevatorRoot", 0.75, 0.51);
    private final LoggedMechanismLigament2d elevatorLigament = new LoggedMechanismLigament2d("elevatorLigament", 0.13, 90.0, 20.0, white);

    public ElevatorVizualizer() {
        root.append(elevatorLigament);
    }

    public void updateViz(double height){
        this.root.setPosition(0.75, 0.51 + height);
    }

    @Override
    public void updateLog(String standardPrefix, String inputPrefix) {
        Logger.recordOutput(standardPrefix + "/ElevatorViz", this.viz2d);
    }
}
