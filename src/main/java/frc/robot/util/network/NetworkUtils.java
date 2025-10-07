// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util.network;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

import edu.wpi.first.wpilibj.DataLogManager;

/** 
 * Utilities file for handling Network operations
 * 
 * Uses may include pulling MAC addresses, Rio IP address, etc.
 */
public final class NetworkUtils {

    // ---- MAC Address Retrieval Functionality ----

    /**
     * Consolidate MAC address related functionality together
     */
    public final class MAC {
        /**
         * retrieve the MAC address of the RoboRIO's networking interface
         * @return The MAC address of the interface formatted as a colon separated string
         */
        public static final String getMACAddress(){
            try{
                final String macAddress = NetworkUtils.MAC._getMACAddress();
                return macAddress;
            } catch(SocketException e){
                DataLogManager.log(String.format("The following exception occurred when attempting to retrieve robot MAC address, %s", e.toString()));
            }

            // If null we failed to retrieve the info somewhere along the way
            return null;
        }

        /**
         * Formats a byte array mac address into an actual string and return it, returns null if the formatting failed
         * @param macBytes String of bytes that represent a hardware MAC address, this can be null
         * @return String or null of the formatted address
         */
        public static final String formatMACAddress(byte[] macBytes){
            if (macBytes == null) return null;

            // Build colon separated string from bytes supplied, used to support non-standard ethernet MAC addresses (just in case)
            StringBuilder sb = new StringBuilder();
            for (int i = 0; i < macBytes.length; i++) {
                sb.append(String.format("%02X%s", macBytes[i], (i < macBytes.length - 1) ? ":" : ""));
            }
            return sb.toString();
        }

        /**
         * Internal method to allow exceptions to be handled by the caller method
         * @return The stringified MAC address of the robot controller's network interface
         * @throws SocketException It is possible an I/O exception may occur when retrieving networking interfaces
         */
        private static final String _getMACAddress() throws SocketException{
            Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
            while(networkInterfaces.hasMoreElements()){
                // Retrieve the network interface itself
                NetworkInterface networkInterface = networkInterfaces.nextElement();
                if(networkInterface == null) continue;

                // Attempt to retrieve the MAC address from the network interface
                return NetworkUtils.MAC.formatMACAddress(networkInterface.getHardwareAddress());
            }

            return null;
        }
    
    }

    // ---- END MAC Address Retrieval Functionality ----

    
}
