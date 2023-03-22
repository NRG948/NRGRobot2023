// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.CAN;

public class ShooterSubsystem extends SubsystemBase {

  private static final double MAX_RPM = 5880.0; // change name if reed doesnt like it
  private static final double RPM_PER_VOLT = 493.9; // Provided by systems, the change in RPM per change in volt. Could be useful.
  private static final double KS = 1.0; // guess

  private static final double BACKSPIN_FACTOR = 0.9; // TODO: determine real backspin factor

  public enum GoalShooterRPM {
    // TODO: get real RPMs
    STOP(0.0),
    HYBRID_RPM(250),
    MID_RPM(1000), // 1000 rpm as proposed by Taiga
    HIGH_RPM(1500);

    private final double rpm;

    GoalShooterRPM(double rpm) {
      this.rpm = rpm;
    }

    /**
     * Returns the RPM corresponding to the desired goal.
     * 
     * @return The RPM corresponding to the desired goal.
     */
    private double getRPM() {
      return rpm;
    }
  }

  private static CANSparkMax topShooterMotor = new CANSparkMax(CAN.SparkMax.TOP_SHOOTER, MotorType.kBrushless);
  private static CANSparkMax bottomShooterMotor = new CANSparkMax(CAN.SparkMax.BOTTOM_SHOOTER, MotorType.kBrushless);
  private static RelativeEncoder encoder = topShooterMotor.getEncoder();

  private GoalShooterRPM currentGoalRPM = GoalShooterRPM.STOP;
  
  private double currentRPM;
  private boolean isEnabled = false;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
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
  public void setMotorVoltage(double voltage) {
    bottomShooterMotor.setVoltage(voltage);
    topShooterMotor.setVoltage(voltage * BACKSPIN_FACTOR); // spin bottom motor slower to create backspin
  }
  
  /**
   * Enables the shooter.
   * 
   * @param goalShooterRPM The desired shooter RPM.
   */
  public void enable(GoalShooterRPM goalShooterRPM) {
    isEnabled = true;
    setGoalRPM(goalShooterRPM);
  }
  
  /**
   * Calculates the motor voltage needed for a given RPM.
   * 
   * @param goalShooterRPM The desired RPM.
   * @return The motor voltage needed.
   */
  public double calculateMotorVoltage(GoalShooterRPM goalShooterRPM) {
    return goalShooterRPM.getRPM() / RPM_PER_VOLT + KS; // not sure
  }
  
  /**
   * Stops the motors.
   */
  public static void stopMotor() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }
  
  /**
   * Disable the shooter.
   */
  public void disable() {
    isEnabled = false;
    stopMotor();
  }

  /**
   * Returns the current RPM.
   * 
   * @return the current RPM.
   */
  public double getCurrentRPM() {
    return currentRPM;
  }
  
  /**
   * Returns the current goal RPM.
   * 
   * @return the current goal RPM.
   */
  public GoalShooterRPM getCurrentGoalRPM() {
    return currentGoalRPM;
  }

  /**
   * Returns whether the shooter is enabled.
   * 
   * @return whether the shooter is enabled.
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  public void periodic() {
    currentRPM = encoder.getVelocity();

    if (isEnabled) {
      setMotorVoltage(calculateMotorVoltage(currentGoalRPM));
    }
  }

}
