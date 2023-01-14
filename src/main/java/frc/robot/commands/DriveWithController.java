// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithController extends CommandBase {

  private SwerveSubsystem swerveDrive;
  private CommandXboxController driveController;

  /** Creates a new DriveWithController. */
  public DriveWithController(SwerveSubsystem sDrive, CommandXboxController dController) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = sDrive;
    driveController = dController;
    addRequirements(swerveDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  // TODO? : add tolerance value to execute() so it does not
  @Override
  public void execute() {
    swerveDrive.drive(-driveController.getLeftY(), -driveController.getLeftX(), -driveController.getRightX(), true,
        false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("END DriveWithController interrupted:" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}