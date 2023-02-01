// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The claw subsystem is responsible for grabbing and holding onto the game
 * elements for delivery to the scoring grids.
 */
public class ClawSubsystem extends SubsystemBase {

  /** An enumeration of possible claw positions. */
  public enum Position {
    // three action instances for the claw
    OPEN(0),
    GRAB_CONE(45),
    GRAB_CUBE(90);

    private final double angle;

    /**
     * Creates a Position instance.
     * 
     * @param angle The servo motor angle.
     */
    private Position(double angle) {
      this.angle = angle;
    }

    /**
     * Returns the servo motor angle.
     * 
     * @return
     */
    private double getAngle() {
      return angle;
    }
  };

  private final Servo servo = new Servo(1);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    set(Position.OPEN);
  }

  /**
   * Sets the claw position.
   * 
   * @param action The claw position.
   */
  public void set(Position action) {
    servo.setAngle(action.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
