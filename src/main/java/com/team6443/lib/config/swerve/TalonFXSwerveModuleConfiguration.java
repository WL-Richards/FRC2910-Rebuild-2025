// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.config.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.config.encoders.CANCoderConfiguration;
import com.team6443.lib.motors.interfaces.MotorIO.NeutralMode;

/** Implementation of the swerve module configuration when using 2 TalonFX motor controllers and one CANCoder */
public class TalonFXSwerveModuleConfiguration extends SwerveModuleConfiguration<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> {
    
    public TalonFXSwerveModuleConfiguration(
        String moduleName, 
        CANDeviceID driveMotorID, 
        CANDeviceID steerMotorID, 
        CANDeviceID steerEncoderID
    ){
       super(moduleName, driveMotorID, steerMotorID, steerEncoderID);
    }

    public TalonFXSwerveModuleConfiguration(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> constants,
        String moduleName, 
        CANDeviceID driveMotorID, 
        CANDeviceID steerMotorID, 
        CANDeviceID steerEncoderID
    ){
        super(moduleName, driveMotorID, steerMotorID, steerEncoderID);
        this.kModuleConstants = constants;
        this.kDriveMotorConfiguration = constants.DriveMotorInitialConfigs;
        this.kSteerMotorConfiguration = constants.SteerMotorInitialConfigs;
    }
    
    @Override
    public SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> getModuleConstants() {
        if(kModuleConstants == null){
            double couplingGearRatio = (double)kDriveGearBox.getStage(0).getGearRatio();
            kModuleConstants = new SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                .withDriveMotorId(kDriveMotor.getDeviceID())
                .withSteerMotorId(kSteerMotor.getDeviceID())
                .withEncoderId(kSteerEncoder.getDeviceID())
                .withDriveMotorGearRatio(kDriveGearBox.getTotalRatio())
                .withSteerMotorGearRatio(kSteerGearBox.getTotalRatio())
                .withCouplingGearRatio(couplingGearRatio)
                .withDriveMotorInverted(kDriveMotorInverted)
                .withSteerMotorInverted(kSteerMotorInverted)
                .withEncoderInverted(kSteerEncoderInverted)
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
    public TalonFXConfiguration getDriveMotorConfiguration() {
       if(kDriveMotorConfiguration == null){
        kDriveMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(kDriveMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(kDriveMotorStatorCurrentLimit)
            )
            .withMotorOutput(
                new MotorOutputConfigs().withNeutralMode(kDriveNeutralMode == NeutralMode.BRAKE 
                                                            ? NeutralModeValue.Brake 
                                                            : NeutralModeValue.Coast
                                                        )
            );
       }
       return kDriveMotorConfiguration;
    }

    @Override
    public TalonFXConfiguration getSteerMotorConfiguration() {
        if(kSteerMotorConfiguration == null){
            kSteerMotorConfiguration = new TalonFXConfiguration()
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withStatorCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(kSteerMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(kSteerMotorStatorCurrentLimit)
                )
                .withMotorOutput(
                    new MotorOutputConfigs().withNeutralMode(kSteerNeutralMode == NeutralMode.BRAKE 
                                                                ? NeutralModeValue.Brake 
                                                                : NeutralModeValue.Coast
                                                            )
                );
        }
        return kSteerMotorConfiguration;
    }

    @Override
    public CANcoderConfiguration getSteerEncoderConfiguration() {
        if(kSteerEncoderConfiguration == null){
            kSteerEncoderConfiguration = new CANcoderConfiguration();
        }
        return kSteerEncoderConfiguration;
    }

}
