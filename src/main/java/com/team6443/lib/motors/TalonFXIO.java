// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com.team6443.lib.motors;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team6443.lib.can.CANDeviceID;
import com.team6443.lib.can.interfaces.CANable;
import com.team6443.lib.motors.interfaces.MotorIO;

/** 
 * Generic implementation for the Talon FX as a motor
*/
public class TalonFXIO implements MotorIO, CANable {
    private TalonFX talon;
    private CANDeviceID deviceID;

    public TalonFXIO(CANDeviceID device){
        this.deviceID = device;
    }

    /**
     * Get a reference to underlying motor itself
     * @return
     */
    public TalonFX getTalon(){
        return talon;
    }

    @Override
    public CANDeviceID getCANDevice() {
        return this.deviceID;
    }

    @Override
    public boolean setEnableSoftwareLimits(boolean forwardLimitEnabled, boolean reversLimitEnabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setEnableSoftwareLimits'");
    }

    @Override
    public boolean setEnableHardwareLimits(boolean forwardLimitEnabled, boolean reversLimitEnabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setEnableHardwareLimits'");
    }

    @Override
    public boolean setZeroOnHardwareLimit(boolean forwardLimitEnabled, boolean reversLimitEnabled) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setZeroOnHardwareLimit'");
    }

    @Override
    public boolean follow(CANDeviceID masterDevice, FollowDirection direction) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'follow'");
    }

    @Override
    public void setSmartMotorConfig(MotionMagicConfigs config) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSmartMotorConfig'");
    }

    @Override
    public void setVoltageConfig(VoltageConfigs config) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltageConfig'");
    }

    @Override
    public boolean setCurrentEncoderPosition(double position) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setCurrentEncoderPosition'");
    }

    @Override
    public boolean updateInputs(MotorInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public boolean setVoltageOutput(double volts) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setVoltageOutput'");
    }

    @Override
    public boolean setTorqueCurrent(double current) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTorqueCurrent'");
    }

    @Override
    public boolean setOpenLoopDutyCylce(double dutyCycle) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setOpenLoopDutyCylce'");
    }

    @Override
    public boolean setNeutralMode(NeutralMode mode) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setNeutralMode'");
    }

    @Override
    public boolean setPIDPositionSetpoint(double posSetpoint) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setPIDPositionSetpoint'");
    }

    @Override
    public boolean setSmartPositionSetpoint(double posSetpoint, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSmartPositionSetpoint'");
    }

    @Override
    public boolean setDynamicSmartPositionSetpoint(double posSetpoint, double velocity, double acceleration,
            double jerk, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setDynamicSmartPositionSetpoint'");
    }

    @Override
    public boolean setSmartVelocitySetpoint(double unitsPerSecond, int slot) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setSmartVelocitySetpoint'");
    }
}
