// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.opencv.photo.Photo;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithAutoOrientation extends CommandBase {

  private final SwerveSubsystem drivetrain;
  private final PhotonVisionSubsystem vision;
  private final CommandXboxController driveController;
  private ProfiledPIDController controller;

  /** Creates a new DriveWithAutoOrientation. */
  public DriveWithAutoOrientation(SwerveSubsystem drivetrain, PhotonVisionSubsystem vision, CommandXboxController driveController) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new ProfiledPIDController(1.0, 0, 0, drivetrain.getRotationalConstraints());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d currentOrientation = drivetrain.getOrientation();
    Rotation2d targetOrientation = currentOrientation;
    if(vision.hasTargets()) {
      Rotation2d angleToTarget = Rotation2d.fromDegrees(vision.getAngleToBestTarget());
      targetOrientation = targetOrientation.plus(angleToTarget);
    }
    double rotationalSpeed = controller.calculate(currentOrientation.getRadians(), targetOrientation.getRadians());
    drivetrain.drive(-driveController.getLeftY(), driveController.getLeftX(), rotationalSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
