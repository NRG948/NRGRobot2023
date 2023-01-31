// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  // TODO: Need to assign proper CAN ID's
  private static final int FRONT_MOTOR_ID = 100;
  private static final int SIDE_MOTOR_ID = 101;
  private static final double INTAKE_POWER = 0.3;

  private CANSparkMax frontMotor;
  private CANSparkMax sideMotor;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    frontMotor = new CANSparkMax(FRONT_MOTOR_ID, MotorType.kBrushless);
    sideMotor = new CANSparkMax(SIDE_MOTOR_ID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {}

  public void acquire() {
    frontMotor.set(INTAKE_POWER);
    sideMotor.set(INTAKE_POWER);
  }

  public void stop() {
    frontMotor.stopMotor();
    sideMotor.stopMotor();
  }
}
