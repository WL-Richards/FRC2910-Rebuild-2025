// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.logging.interfaces;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.team6443.frc2025.constants.generated.BuildConstants;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/** 
 * Provides for how the object responsible for logging the data should look
*/
public interface Loggerable {

    /**
     * Sets up the logger to log to log to the correct source and then starts it
     */
    default public void setupLogger() {

        // Update the metadata prior to starting the logger, THIS IS A REQUIREMENT
        updateMetadata();
        
        /* --- On real robot log to USB drive and NetworkTables (Only if NOT attached to FMS, this allows for faster loop times) --- */
        if(RobotBase.isReal()){
            Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
            if (!DriverStation.isFMSAttached()) {
                Logger.addDataReceiver(new NT4Publisher());
            }
        }
        
        /* --- On simulated bot log to both advantage kit --- */
        else if(RobotBase.isSimulation()){
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        }

        /* --- On replay bot do other stuff TODO: Implement --- */

        // Begin logging
        Logger.start();
    };

    /**
     * Update the metadata in the logger with some build information by default
     * 
     * This leverages gversion to generate a constants file on build 
     */
    public default void updateMetadata(){
        Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
        Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
        Logger.recordMetadata("DeployingUser", BuildConstants.GIT_EMAIL);
        switch (BuildConstants.DIRTY) {
            case 0:
                Logger.recordMetadata("GitDirty", "All changes committed");
                break;
            case 1:
                Logger.recordMetadata("GitDirty", "Uncommitted changes");
                break;
            default:
                Logger.recordMetadata("GitDirty", "Unknown");
                break;
        }
    };
}
