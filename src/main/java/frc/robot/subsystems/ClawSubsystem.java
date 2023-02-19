// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants.PWMPort;

/**
 * The claw subsystem is responsible for grabbing and holding onto the game
 * elements for delivery to the scoring grids.
 */
public class ClawSubsystem extends SubsystemBase {

  /** An enumeration of possible claw positions. */
  public enum Position {
    // three position instances for the claw
    OPEN(90),
    GRAB_CONE(60),
    GRAB_CUBE(70);

    private final double angle;  // The servo angle (0-180 degree range)

    /**
     * Creates a Position instance.
     * 
     * @param angle The servo motor angle.
     */
    private Position(double angle) {
      this.angle = angle;
    }

    /**
     * Returns the servo motor angle in degrees (0-180).
     * 
     * @return
     */
    private double getAngle() {
      return angle;
    }
  };

  private final Servo servo = new Servo(PWMPort.SERVO);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    set(Position.OPEN);
  }

  /**
   * Sets the claw servo to a desired claw position.
   * 
   * @param position The claw position.
   */
  public void set(Position position) {
    servo.setAngle(position.getAngle());
    System.out.println("Servo Angle: " + position.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
