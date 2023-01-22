// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Set;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ProfiledDriveStraight extends CommandBase {
  private final SwerveSubsystem drivetrain;
  private final double distance;
  private final double heading;
  private final SwerveDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private final TrapezoidProfile profile;
  private final Timer timer = new Timer();
  private Pose2d initialPose;

  /** Creates a new ProfiledDriveStraight. */
  public ProfiledDriveStraight(
      SwerveSubsystem drivetrain,
      HolonomicDriveController controller,
      double distance,
      double heading) {
    this.drivetrain = drivetrain;
    this.distance = distance;
    this.heading = heading;
    this.kinematics = SwerveSubsystem.kKinematics;
    this.controller = drivetrain.createDriveController();
    this.profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(drivetrain.getMaxSpeed(), drivetrain.getMaxAcceleration()),
        new TrapezoidProfile.State(distance, 0));
  }

  @Override
  public void initialize() {
    initialPose = drivetrain.getPosition();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    TrapezoidProfile.State state = profile.calculate(timer.get());
    Translation2d offset = new Translation2d(state.position, Rotation2d.fromDegrees(heading));
    Pose2d nextPose = initialPose.plus(new Transform2d(offset, initialPose.getRotation()));
    ChassisSpeeds speeds = controller.calculate(drivetrain.getPosition(), nextPose, state.velocity,
        initialPose.getRotation());
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    drivetrain.setModuleStates(moduleStates);
  }

  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
  }
}
