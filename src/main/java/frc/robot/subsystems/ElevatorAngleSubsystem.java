// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;
import frc.robot.parameters.MotorParameters;

/**
 * The elevator angle subsystem is responsible for controlling the elevator
 * angle.
 */
public class ElevatorAngleSubsystem extends SubsystemBase {

  // AQUIRING and SCORING are out of the frame perimeter
  public enum ElevatorAngle {
    ACQUIRING(30),
    STOWED(90),
    SCORING(120);

    private final double angle;

    private ElevatorAngle(double angle) {
      this.angle = angle;
    }

    /** Returns elevator angle in degrees.*/
    private double getAngle() {
      return angle;
    }
  }

  // CONSTANTS
  private final double ANGLE_RANGE = 90;
  private final double GEAR_RATIO = 4 / 1;
  private final double MOTOR_POWER = 0.3;

  // Calculate degrees per pulse
  private final double DEGREES_PER_REVOLUTION = ANGLE_RANGE / (GEAR_RATIO * 360);

  private final CANSparkMax motor = new CANSparkMax(CAN.SparkMax.ELEVATOR_ANGLE, MotorType.kBrushless);
  private final RelativeEncoder encoder = motor.getAlternateEncoder(MotorParameters.NeoV1_1.getPulsesPerRevolution());
  private double angleOffset; // record encoder's current position
  private ElevatorAngle goalAngle = ElevatorAngle.STOWED;
  private double motorPower = 0;
  private double currentAngle = ElevatorAngle.STOWED.getAngle(); // start as stowed
  private boolean isPeriodicControlEnabled = false;

  /** Creates a new ElevatorAngleSubsystem. */
  public ElevatorAngleSubsystem() {
    // convert encoder ticks to angle
    encoder.setPositionConversionFactor(DEGREES_PER_REVOLUTION);
    angleOffset = encoder.getPosition() - ElevatorAngle.STOWED.getAngle();
  }

  public void setMotor(double power) {
    if (power == 0) {
      motor.stopMotor();
    } else {
      motor.set(power);
    }
  }

  /**
   * Sets the goal elevator angle.
   * 
   * @param goalAngle the goal elevator angle.
   */
  public void setGoalAngle(ElevatorAngle goalAngle) {
    this.goalAngle = goalAngle;
    // Acquiring to scoring -> Positive motor power
    // Scoring to acquiring -> Negative motor power
    this.motorPower = Math.signum(goalAngle.getAngle() - currentAngle) * MOTOR_POWER;
    isPeriodicControlEnabled = true;
  }

  /**
   * Returns whether the elevator is at the goal angle.
   * 
   * @return True if the elevator is at the goal angle.
   */
  public boolean atGoalAngle() {
    return motorPower > 0 ? currentAngle >= goalAngle.getAngle() : currentAngle <= goalAngle.getAngle();
  }

  /**
   * Gets the current elevator angle in degrees.
   * 
   * @return the current elevator angle.
   */
  public double getAngle() {
    return currentAngle;
  }

  /** Enables periodic control. */
  public void enablePeriodicControl(boolean isEnabled){
    isPeriodicControlEnabled = isEnabled;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!isPeriodicControlEnabled){
      return;
    }
    currentAngle = encoder.getPosition() - angleOffset;

    // Shut off motor if at the desired angle.
    if (atGoalAngle()) {
      motorPower = 0;
      isPeriodicControlEnabled = false;
    }
    setMotor(motorPower);
  }

}
