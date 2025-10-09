// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.config.base.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import frc.robot.util.can.CANDeviceID;
import frc.robot.util.config.NamedCANCoderConfiguration;
import frc.robot.util.config.NamedTalonFXConfiguration;

/** Implementation of the swerve module configuration when using 2 TalonFX motor controllers and one CANCoder */
public class TalonFXSwerveModuleConfiguration extends SwerveModuleConfiguration<NamedTalonFXConfiguration, NamedTalonFXConfiguration, NamedCANCoderConfiguration> {
    
    public TalonFXSwerveModuleConfiguration(String moduleName, CANDeviceID driveMotorID, CANDeviceID steerMotorID, CANDeviceID steerEncoderID){
       super(moduleName, driveMotorID, steerMotorID, steerEncoderID);
    }
    
    @Override
    public SwerveModuleConstants<NamedTalonFXConfiguration, NamedTalonFXConfiguration, NamedCANCoderConfiguration> getModuleConstants() {
        if(kModuleConstants == null){
            double couplingGearRatio = (double)kDriveGearBox.getStage(0).getGearRatio();
            kModuleConstants = new SwerveModuleConstants<NamedTalonFXConfiguration, NamedTalonFXConfiguration, NamedCANCoderConfiguration>()
                .withDriveMotorId(kDriveMotor.getDeviceID())
                .withSteerMotorId(kSteerMotor.getDeviceID())
                .withEncoderId(kSteerEncoder.getDeviceID())
                .withDriveMotorGearRatio(kDriveGearBox.getTotalRatio())
                .withSteerMotorGearRatio(kSteerGearBox.getTotalRatio())
                .withCouplingGearRatio(couplingGearRatio)
                .withDriveMotorInverted(false)
                .withSteerMotorInverted(false)
                .withEncoderInverted(false)
                .withEncoderOffset(kSteerEncoderOffsetRotations)
                .withLocationX(kLocationOffset.getX())
                .withLocationY(kLocationOffset.getY())
                .withDriveMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withSteerMotorClosedLoopOutput(SwerveModuleConstants.ClosedLoopOutputType.Voltage)
                .withDriveMotorGains(kDriveMotorGains)
                .withSteerMotorGains(kSteerMotorGains)
                .withDriveMotorType(SwerveModuleConstants.DriveMotorArrangement.TalonFX_Integrated)
                .withSteerMotorType(SwerveModuleConstants.SteerMotorArrangement.TalonFX_Integrated)
                .withDriveMotorInitialConfigs(kDriveMotorConfiguration)
                .withSteerMotorInitialConfigs(kSteerMotorConfiguration)
                .withEncoderInitialConfigs(kSteerEncoderConfiguration)
                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                .withDriveInertia(kDriveInertia)
                .withSteerInertia(kSteerInertia)
                .withSlipCurrent(kDriveMotorSlipCurrent)
                .withFeedbackSource(SwerveModuleConstants.SteerFeedbackType.FusedCANcoder)
                .withSpeedAt12Volts(kMaxRobotSpeedMeterPerSecond)
                .withWheelRadius(kWheelRadiusM);
        }
        return kModuleConstants;
    }

    @Override
    public NamedTalonFXConfiguration getDriveMotorConfiguration() {
       if(kDriveMotorConfiguration == null){
        kDriveMotorConfiguration = (new NamedTalonFXConfiguration(moduleName + "_DriveMotor", driveMotorID));
        kDriveMotorConfiguration.withCurrentLimits(
            new CurrentLimitsConfigs()
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(kDriveMotorSupplyCurrentLimit)
                .withStatorCurrentLimit(kDriveMotorStatorCurrentLimit)
        );
       }
       return kDriveMotorConfiguration;
    }

    @Override
    public NamedTalonFXConfiguration getSteerMotorConfiguration() {
        if(kSteerMotorConfiguration == null){
            kSteerMotorConfiguration = new NamedTalonFXConfiguration(moduleName + "_SteerMotor", steerMotorID);
            kSteerMotorConfiguration.withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(kSteerMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(kSteerMotorStatorCurrentLimit)
            );
        }
        return kSteerMotorConfiguration;
    }

    @Override
    public NamedCANCoderConfiguration getSteerEncoderConfiguration() {
        if(kSteerEncoderConfiguration == null){
            kSteerEncoderConfiguration = new NamedCANCoderConfiguration(moduleName + "_SteerEncoder", steerEncoderID);
        }
        return kSteerEncoderConfiguration;
    }

}
