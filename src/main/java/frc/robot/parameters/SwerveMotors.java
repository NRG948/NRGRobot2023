// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.parameters;

/** Add your docs here. */
public enum SwerveMotors {
  FrontLeftDrive, FrontLeftSteering,
  FrontRightDrive, FrontRightSteering,
  BackLeftDrive, BackLeftSteering,
  BackRightDrive, BackRightSteering;

  /**
   * Returns the index of the Talon CAN ids.
   */
  public int getIndex() {
    return this.ordinal();
  }
}
