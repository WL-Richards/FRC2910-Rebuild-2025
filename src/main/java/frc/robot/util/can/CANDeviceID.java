// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.can;

/**
 * Abstraction layer for representing some device that uses CAN to communicate and has both a CAN ID and a CAN BUS
 * 
 * Largely yoinked from 2910 (Jack in the Bot): https://github.com/FRCTeam2910/2025CompetitionRobot-Public/blob/main/src/main/java/org/frc2910/robot/util/CanDeviceId.java
 */
public class CANDeviceID {
    private final int canID;
    private final String busName;
    
    public CANDeviceID(int canID, String busName){
        this.canID = canID;
        this.busName = busName;
    }

    // Use the default bus name "rio".
    public CANDeviceID(int canID) {
        this(canID, "rio");
    }

    public int getDeviceID() {
        return canID;
    }

    public String getBus() {
        return busName;
    }

    public boolean equals(CANDeviceID other) {
        return other.canID == canID && other.busName.equals(busName);
    }

    @Override
    public String toString() {
        return "CANDeviceID(" + canID + ", " + busName + ")";
    }
}
