// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IndexerSubsystem;

public class IndexByController extends CommandBase {
 private CommandXboxController controller;
 private IndexerSubsystem indexSubsystem;

  private static final double DEADBAND = 0.1;
  private static final double INDEX_SPEED = 0.5;

  public IndexByController(IndexerSubsystem indexSubsystem, CommandXboxController controller) {
    this.controller = controller;
    this.indexSubsystem = indexSubsystem;
    addRequirements(indexSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    indexSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getHID().getRightY();
    speed = MathUtil.applyDeadband(speed * INDEX_SPEED, DEADBAND);
    indexSubsystem.runMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    indexSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
