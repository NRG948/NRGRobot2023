// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.ElevatorAngleSubsystem;

public class TiltElevatorWithController extends CommandBase {
  private ElevatorAngleSubsystem elevatorAngle;
  private CommandXboxController controller;
  private static final double MAX_POWER = 0.7;

  /**
   * Creates a new TiltElevatorWithController.
   * 
   * This command uses the right joystick input of the controller to change the
   * angle of the elevator. It is intended for use as the default command of
   * {@link ElevatorAngleSubsystem}, but should only be used for testing.
   * 
   * @param elevatorAngle The elevator angle subsystem.
   * @param controller    The Xbox controller used to adjust the elevator angle.
   */
  public TiltElevatorWithController(ElevatorAngleSubsystem elevatorAngle, CommandXboxController controller) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevatorAngle = elevatorAngle;
    this.controller = controller;
    addRequirements(elevatorAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorAngle.enablePeriodicControl(false);
  }

  /**
   * Called every time the scheduler runs while the command is scheduled.
   * Converts the right joystick input of manipulator to adjst the elevator angle.
   */
  @Override
  public void execute() {
    double speed = controller.getRightX();
    speed = MathUtil.applyDeadband(speed, 0.05) * MAX_POWER;
    elevatorAngle.setMotorVoltage(speed * RobotConstants.MAX_BATTERY_VOLTAGE);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorAngle.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
