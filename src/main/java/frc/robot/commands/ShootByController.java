// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootByController extends CommandBase {
 private CommandXboxController controller;
 private IntakeSubsystem shooterSubsystem;

  private static final double DEADBAND = 0.1;
  private static final double SHOOTER_SPEED = 0.85; //TO-DO: Find constant shooter speed based on the manual

  public ShootByController(IntakeSubsystem intakeSubsystem, CommandXboxController controller) {
    this.controller = controller;
    this.shooterSubsystem = shooterSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getHID().getRightTriggerAxis();// Change for shooter NEOs
    speed = MathUtil.applyDeadband(speed * SHOOTER_SPEED, DEADBAND);
    shooterSubsystem.runMotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stopMotor();

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
