// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

public class ShooterSubsystem extends SubsystemBase {

  private static final double MAX_RPM = 500.0; // change name if reed doesnt like it
  private static final double RPM_PER_VOLT = 493.9; // Provided by systems, the change in RPM per change in volt. Could be useful.
  private static final double INDEXER_RPM = 200.0; // TODO: get real RPMs

  private static final double BACKSPIN_FACTOR = 0.9; // TODO: determine real backspin factor

  public enum GoalShooterRPM {
    // TODO: get real RPMs
    STOP(0.0),
    HYBRID_RPM_PERCENT(0.2),
    MID_RPM_PERCENT(0.45),
    HIGH_RPM_PERCENT(0.75);

    private final double rpm;

    GoalShooterRPM(double rpm) {
      this.rpm = rpm;
    }

    /**
     * Returns the current RPM of the shooter.
     * 
     * @return The current RPM of the scooter.
     */
    private double getGoalShooterRPM() {
      return rpm;
    }
  }


  private static CANSparkMax indexerMotor = new CANSparkMax(CAN.SparkMax.INDEXER, MotorType.kBrushless);
  private static CANSparkMax topShooterMotor = new CANSparkMax(CAN.SparkMax.TOP_SHOOTER, MotorType.kBrushless);
  private static CANSparkMax bottomShooterMotor = new CANSparkMax(CAN.SparkMax.BOTTOM_SHOOTER, MotorType.kBrushless);

  private GoalShooterRPM currentGoalRPM = GoalShooterRPM.STOP;
  private boolean isEnabled = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    indexerMotor.setIdleMode(IdleMode.kBrake);
    topShooterMotor.setIdleMode(IdleMode.kBrake);
    bottomShooterMotor.setIdleMode(IdleMode.kBrake);
  }

  public void periodic() {
    if (isEnabled) {
      setIndexerMotorPower(INDEXER_RPM);
      setShooterMotorPower(currentGoalRPM.getGoalShooterRPM());
    }
  }

  /**
   * Sets the goal RPM of the shooter.
   * 
   * @param goalRPM The goal RPM.
   */
  public void setGoalRPM(GoalShooterRPM goalRPM) {
    currentGoalRPM = goalRPM;
  }

  /**
   * Sets the motor speed for the shooter motors.
   * 
   * @param speed The desired motor speed.
   */
  public void setShooterMotorPower(double speed) {
    bottomShooterMotor.set(speed);
    topShooterMotor.set(speed * BACKSPIN_FACTOR); // spin bottom motor slower to create backspin
  }

  /**
   * Sets the motor speed for the indexer motor.
   * 
   * @param speed The desired motor speed.
   */
  public void setIndexerMotorPower(double speed) {
    indexerMotor.set(speed);
  }

  public void shoot(GoalShooterRPM GoalRPM) {
    isEnabled = true;
    currentGoalRPM = GoalRPM;
  }

  public static void stopMotor() {
    indexerMotor.stopMotor();
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  public void disable() {
    isEnabled = false;
    stopMotor();
  }
}
