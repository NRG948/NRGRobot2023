// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The acquirer subsystem is responsable for picking up game elements from the
 * floor.
 */
public class AcquirerSubsystem extends SubsystemBase {
  private static final double MOTOR_SPEED = 0.3;
  private CANSparkMax motor1 = new CANSparkMax(103, MotorType.kBrushless);
  private CANSparkMax motor2 = new CANSparkMax(104, MotorType.kBrushless);

  private boolean isEnabled = false;

  /** Creates a new AcquirerSubsystem. */
  public AcquirerSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (isEnabled) {
      motor1.set(MOTOR_SPEED);
      motor2.set(MOTOR_SPEED);
    } else {
      motor1.stopMotor();
      motor2.stopMotor();
    }
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
