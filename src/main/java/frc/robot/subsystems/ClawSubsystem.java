// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {

  public enum Action {
    // three action instances for the claw
    OPEN (0),
    GRAB_CONE (45),
    GRAB_CUBE (90);

    private final double angle;

    private Action(double angle) {
      this.angle = angle;
    }

    private double getAngle() {
      return angle;
    }
  };

  private final Servo servo = new Servo(1);

  /** Creates a new ClawSubsystem. */
  public ClawSubsystem() {
    set(Action.OPEN);
  }

  public void set(Action action) {
    servo.setAngle(action.getAngle());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
