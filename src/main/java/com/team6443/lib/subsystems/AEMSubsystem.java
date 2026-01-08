// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.subsystems;

import com.team6443.lib.core.logging.Loggable;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public abstract class AEMSubsystem extends SubsystemBase implements Loggable {

  // Name of the subsystem that is in use
  public final String kSubsystemName;

  // Used for standard logs
  protected final String kLogPrefixStandard;

  // Use for input logs
  protected final String kLogPrefixInput;

  /** Base level Subsystem that handles basic logging functionality */
  public AEMSubsystem(String name) {
    this.kSubsystemName = name;

    this.kLogPrefixStandard = "Subsystems/" + this.kSubsystemName;
    this.kLogPrefixInput = "RealOutputs/" + kLogPrefixStandard;
  }

  @Override
  public void updateLog(){
    this.updateLog(this.kLogPrefixStandard, this.kLogPrefixInput);
  }
}
