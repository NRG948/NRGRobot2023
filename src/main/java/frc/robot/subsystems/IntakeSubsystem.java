// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

public class IntakeSubsystem extends SubsystemBase {
  private static final double INTAKE_POWER = 0.3;

  private final CANSparkMax frontMotor = new CANSparkMax(CAN.SparkMax.INTAKE_FRONT, MotorType.kBrushless);
  private final CANSparkMax sideMotor = new CANSparkMax(CAN.SparkMax.INTAKE_SIDE, MotorType.kBrushless);

  private boolean isEnabled = false;

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {

  }

  @Override
  public void periodic() {
    if (isEnabled) {
      frontMotor.set(INTAKE_POWER);
      sideMotor.set(INTAKE_POWER);
    } else {
      frontMotor.stopMotor();
      sideMotor.stopMotor();
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
