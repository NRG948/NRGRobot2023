// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.ElevatorSubsystem;

/**
 * This command uses the left joystick input of the controller to raise and
 * lower the elevator. It is intended for use as the default command of
 * {@link ElevatorSubsystem}, but should only be used for testing.
 */
public class RaiseElevatorWithController extends CommandBase {
  private ElevatorSubsystem elevator;
  private CommandXboxController controller;
  private static final double MAX_POWER = 1.0;

  /**
   * Creates a new ElevatorWithController.
   * 
   * This command uses the left joystick input of the controller to raise and
   * lower the elevator. It is intended for use as the default command of
   * {@link ElevatorSubsystem}, but should only be used for testing.
   * 
   * @param elevator   The elevator subsystem.
   * @param controller The Xbox controller used to raise and lower the elevator.
   */
  public RaiseElevatorWithController(ElevatorSubsystem elevator, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    this.controller = controller;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.disableGoalSeeking();
    System.out.println("BEGIN RaiseElevatorWithController");
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Converts the left joystick input of manipulator to raise/lower the elevator.
   */
  @Override
  public void execute() {
    double speed = -controller.getLeftY();
    speed = MathUtil.applyDeadband(speed, 0.05) * MAX_POWER;
    elevator.setMotorVoltage(speed * RobotConstants.MAX_BATTERY_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopMotor();
    System.out.println("END RaiseElevatorWithController");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
