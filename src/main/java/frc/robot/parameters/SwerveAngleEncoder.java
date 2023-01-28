// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.parameters;

/** Add your docs here. */
public enum SwerveAngleEncoder {
  FrontLeft,
  FrontRight,
  BackLeft,
  BackRight;

  /**
   * Returns the index of the CANcoder ids.
   */
  public int getIndex() {
    return this.ordinal();
  }
}
