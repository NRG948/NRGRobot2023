// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This command uses the joystick inputs of the controller to drive the robot.
 * The left joystick controls translational movement and the right joystick
 * controls rotation of the robot.
 * <p>
 * This command intended for use as the default command of
 * {@link SwerveSubsystem}.
 */
public class DriveWithController extends CommandBase {

  public static final double DEADBAND = 0.1;

  private SwerveSubsystem swerveDrive;
  private CommandXboxController driveController;

  /**
   * Creates a new DriveWithController.
   * 
   * This command uses the joystick inputs of the controller to drive the robot.
   * The left joystick controls translational movement and the right joystick
   * controls rotation of the robot.
   * <p>
   * This command intended for use as the default command of
   * {@link SwerveSubsystem}.
   * 
   * @param sDrive      The swerve drive subsystem.
   * @param dController The driver's Xbox controller.
   */
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
  @Override
  public void execute() {
    double xSpeed = -driveController.getLeftY();
    double ySpeed = -driveController.getLeftX();
    double rSpeed = -driveController.getRightX();
    double inputScalar = Math.max(1-driveController.getRightTriggerAxis(), 0.15);

    // Applies deadbands to x, y, and rotation joystick values and multiples all
    // values with inputScalar which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;
    rSpeed = MathUtil.applyDeadband(rSpeed, DEADBAND) * inputScalar;

    swerveDrive.drive(
        xSpeed,
        ySpeed,
        rSpeed,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("END DriveWithController interrupted:" + interrupted);
    swerveDrive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}