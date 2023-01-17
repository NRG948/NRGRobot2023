// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.drive.SwerveDrive;

public class DriveStraight extends CommandBase {

  private SwerveSubsystem swerveDrive; // creates a new SwerveDrive Subsystem

  private double speed; // speed value

  private double xSpeed; // x-speed
  private double ySpeed; // y-speed

  private double heading; // heading value

  private double orientation; // orientation

  /** Creates a new DriveStraight. */
  public DriveStraight(SwerveSubsystem swerveDrive, double speed, double heading, double orientation) {
    this.swerveDrive = swerveDrive; 
    this.speed = speed;
    this.orientation = orientation;
    addRequirements(swerveDrive); // requires the swerveDrive Subsystem
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double headingRadians = Math.toRadians(heading);
    xSpeed = speed * Math.cos(headingRadians); // use cosine for x
    ySpeed = speed * Math.sin(headingRadians); // use sin for y
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDrive.drive(xSpeed, ySpeed, 0, true, false); // keeping rotational speed @ 0
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDrive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
