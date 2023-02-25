// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

/**
 * The intake subsystem is responsible for acquiring game elements from the
 * floor.
 */
public class IntakeSubsystem extends SubsystemBase {
  private static final double INTAKE_POWER = 0.3; // moving this 

  private final CANSparkMax motor = new CANSparkMax(CAN.SparkMax.INTAKE, MotorType.kBrushless);
  private boolean isEnabled = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
  }

  @Override
  public void periodic() {
    /*if (isEnabled) {
      motor.set(INTAKE_POWER);
    } else {
      motor.stopMotor();
    }*/
  }

  public void runMotor(double power) {
    motor.set(power);
  }

  public void stopMotor() {
    motor.stopMotor();
  }

  /**
   * Enable the intake.
   */
  public void enable() {
    isEnabled = true;
  }

  /**
   * Disable the intake.
   */
  public void disable() {
    isEnabled = false;
  }
}
