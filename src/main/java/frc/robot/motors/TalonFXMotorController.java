// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.motors;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * Adapts a CTRE {@link TalonFX} motor controller to implement the
 * {@link MotorController} interface.
 */
public class TalonFXMotorController implements MotorController {
    private final TalonFX motor;

    /**
     * Constructs an instance of this class.
     * 
     * @param motor The {@link TalonFX} motor controller to adapt to the
     *              {@link MotorController} interface.
     */
    public TalonFXMotorController(TalonFX motor) {
        this.motor = motor;
    }

    @Override
    public void set(double speed) {
        motor.set(ControlMode.PercentOutput, speed);
    }

    @Override
    public double get() {
        return motor.getMotorOutputPercent();
    }

    @Override
    public void setInverted(boolean isInverted) {
        motor.setInverted(isInverted);
    }

    @Override
    public boolean getInverted() {
        return motor.getInverted();
    }

    @Override
    public void disable() {
        motor.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0);
    }
}
