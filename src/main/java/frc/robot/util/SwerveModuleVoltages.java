// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

/** 
 * Represents motor voltages for one swerve module.
 */
public class SwerveModuleVoltages {
  public final double driveVoltage;
  public final double steeringVoltage;
  /**
   * Constructs an instance of this class.
   * 
   * @param driveVoltage The drive voltage.
   * @param steeringVoltage The steering voltage.
   */
  public SwerveModuleVoltages(double driveVoltage, double steeringVoltage){
    this.driveVoltage = driveVoltage;
    this.steeringVoltage = steeringVoltage;
  }
}
