// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config;

/**
 * Represents how all sensors and motors should be mapped to a corresponding port
 */
public final class PortConfiguration {

    /* Configurations specifically related to CAN */
    public final class CAN {
        // Name of the CAN bus
        public String BusName;
        
    }

    // CAN Bus object that is to be configured
    public final PortConfiguration.CAN CANBus = new CAN();

    /**
     * Set the name of the CAN bus that this configuration should use
     * @param busName Stringified CAN bus name
     * @return Reference to this port config
     */
    public PortConfiguration withCANBusName(String busName){
        this.CANBus.BusName = busName;
        return this;
    }
}
