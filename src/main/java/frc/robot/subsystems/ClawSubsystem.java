// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PWMPort;

/**
 * The claw subsystem is responsible for holding and releasing the game pieces
 * for delivery to the scoring grids.
 */
public class ClawSubsystem extends SubsystemBase {

  /** An enumeration of possible claw trapdoor positions. */
  public enum Position {
    OPEN(100),
    CLOSED(60);

    private final double servoAngle;  // The servo angle (0-180 degree range)

    /**
     * Creates a Position instance.
     * 
     * @param servoAngle The servo motor angle.
     */
    private Position(double servoAngle) {
      this.servoAngle = servoAngle;
    }

    /** Returns the servo motor angle in degrees (0-180). */
    private double getServoAngle() {
      return servoAngle;
    }
  };

  private final Servo servo = new Servo(PWMPort.SERVO);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    set(Position.CLOSED);
  }

  /**
   * Sets the trapdoor servo to a desired position.
   * 
   * @param position The desired servo position.
   */
  public void set(Position position) {
    servo.setAngle(position.getServoAngle());
    System.out.println("Servo Angle: " + position.getServoAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
