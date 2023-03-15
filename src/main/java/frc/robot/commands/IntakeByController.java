// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeByController extends CommandBase {
 private CommandXboxController controller;
 private IntakeSubsystem intakeSubsystem;

  private static final double DEADBAND = 0.1;

  public IntakeByController(IntakeSubsystem intakeSubsystem, CommandXboxController controller) {
    this.controller = controller;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.disable();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double speed = controller.getHID().getRightTriggerAxis() - controller.getHID().getLeftTriggerAxis();
    speed = MathUtil.applyDeadband(speed, DEADBAND);
    intakeSubsystem.runMotor(speed);
    System.out.println("INTAKE SPEED: " + speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
