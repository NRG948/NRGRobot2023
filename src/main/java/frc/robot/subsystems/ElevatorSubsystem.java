// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.parameters.MotorParameters;

public class ElevatorSubsystem extends SubsystemBase {

  private CANSparkMax elevatorMotor;
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
     * Returns the desired position of elevator as an int.
     * 
     * @return
     */
    private double getPosition() {
      return position;
    }
  }

  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem() {
    elevatorMotor = new CANSparkMax(102, MotorType.kBrushless);
  }
  
  public void setGoal(GoalState goalState) {
    this.goalState = goalState;
  }
  
  @Override
  public void periodic() {
    currentPosition = elevatorMotor.getAlternateEncoder(MotorParameters.NeoV1_1.getPulsesPerRevolution()).getPosition();
  }
}
