// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
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
public class DriveAndAutoRotate extends CommandBase {

  public static final double DEADBAND = 0.1;

  private final SwerveSubsystem swerveDrive;
  private final CommandXboxController driveController;
  private final double desiredOrientation;
  private final ProfiledPIDController controller;

  /**
   * Creates a new DriveAndAutoRotate.
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
  public DriveAndAutoRotate(SwerveSubsystem sDrive, CommandXboxController dController, double desiredOrientation) {
    // Use addRequirements() here to declare subsystem dependencies.
    swerveDrive = sDrive;
    driveController = dController;
    this.desiredOrientation = desiredOrientation;
    controller = new ProfiledPIDController(1.0, 0, 0, sDrive.getRotationalConstraints());
    controller.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(swerveDrive);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller.setGoal(desiredOrientation);
    controller.reset(swerveDrive.getOrientation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = -driveController.getLeftY();
    double ySpeed = -driveController.getLeftX();
    double rSpeed = controller.calculate(swerveDrive.getOrientation().getRadians());
    double inputScalar = Math.max(1-driveController.getRightTriggerAxis(), 0.15);

    // Applies deadbands to x and y joystick values and multiples all
    // values with inputScalar which allows finer driving control.
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;

    swerveDrive.drive(
        xSpeed,
        ySpeed,
        rSpeed,
        true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("END DriveAndAutoRotate interrupted:" + interrupted);
    swerveDrive.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}