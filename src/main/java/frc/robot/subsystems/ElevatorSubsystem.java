// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.MotorParameters;

/**
 * The elevator subsystem is responsible for setting the claw position for
 * acquiring or scoring game elements.
 */
public class ElevatorSubsystem extends SubsystemBase {

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;

  private GoalState goalState;
  private double currentPosition;

  public enum GoalState {
    ACQUIRE(0),
    SCORE_LOW(100),
    SCORE_MID(200),
    SCORE_HIGH(300);

    private final double position;

    GoalState(double position) {
      this.position = position;
    }

    /**
     * Returns the desired position of elevator.
     * 
     * @return The current position of the elevator.
     */
    private double getPosition() {
      return position;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    motor = new CANSparkMax(102, MotorType.kBrushless);
    encoder = motor.getAlternateEncoder(MotorParameters.NeoV1_1.getPulsesPerRevolution());
  }

  /**
   * Sets the claw position.
   * 
   * @param action The claw position.
   */
  public void setGoal(GoalState goalState) {
    this.goalState = goalState;
  }

  @Override
  public void periodic() {
    currentPosition = encoder.getPosition();
  }
}
