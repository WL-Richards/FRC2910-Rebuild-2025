// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.odometry;

/** 
 * Class used to represent odometry standard deviations
 */
public class OdometryStandardDevs {
    public final double xStdDev;
    public final double yStdDev;
    public final double rotStdDev;

    public OdometryStandardDevs(
        double xStdDevM,
        double yStdDevM,
        double rotStdDevM
    ){
        this.xStdDev =xStdDevM;
        this.yStdDev = yStdDevM;
        this.rotStdDev = rotStdDevM;
    }
}
