// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CubeVisionSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

@RobotPreferencesLayout(groupName = "Cube Vision", row = 0, column = 4, width = 2, height = 3)
public class DriveAndOrientToCube extends CommandBase {
  public static final double DEADBAND = 0.1;
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue kP = new RobotPreferences.DoubleValue(
      "Cube Vision", "kP", 0.1);
      @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue kI = new RobotPreferences.DoubleValue(
      "Cube Vision", "kI", 0);
      @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue kD = new RobotPreferences.DoubleValue(
      "Cube Vision", "kD", 0);


  private final SwerveSubsystem drivetrain;
  private final CubeVisionSubsystem vision;
  private final CommandXboxController driveController;
  private ProfiledPIDController controller;

  /** Creates a new DriveWithAutoOrientation. */
  public DriveAndOrientToCube(SwerveSubsystem drivetrain, CubeVisionSubsystem vision,
      CommandXboxController driveController) {
    this.drivetrain = drivetrain;
    this.vision = vision;
    this.driveController = driveController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    controller = new ProfiledPIDController(kP.getValue(), kI.getValue(), kD.getValue(), drivetrain.getRotationalConstraints()); // p was 1.0
    controller.setTolerance(1.5);
    controller.reset(drivetrain.getOrientation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Rotation2d currentOrientation = drivetrain.getOrientation();
    Rotation2d targetOrientation = currentOrientation;
    double rotationalSpeed = 0;

    double xSpeed = -driveController.getLeftY();
    double ySpeed = -driveController.getLeftX();
    double inputScalar = Math.max(1-driveController.getRightTriggerAxis(), 0.15);
    
    xSpeed = MathUtil.applyDeadband(xSpeed, DEADBAND) * inputScalar;
    ySpeed = MathUtil.applyDeadband(ySpeed, DEADBAND) * inputScalar;

    if (vision.hasTargets()) {
      Rotation2d angleToTarget = Rotation2d.fromDegrees(-vision.getAngleToBestTarget());

      targetOrientation = targetOrientation.plus(angleToTarget);
      rotationalSpeed = controller.calculate(currentOrientation.getRadians(), targetOrientation.getRadians());
    }


    drivetrain.drive(xSpeed, ySpeed, rotationalSpeed, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
