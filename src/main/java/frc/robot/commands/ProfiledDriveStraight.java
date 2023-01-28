// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A command to drive the robot on a straight line in using trapezoidal motion
 * profiling.
 */
public class ProfiledDriveStraight extends CommandBase {
  private final SwerveSubsystem drivetrain;
  private final Rotation2d heading;
  private final SwerveDriveKinematics kinematics;
  private final HolonomicDriveController controller;
  private final TrapezoidProfile profile;
  private final Timer timer = new Timer();
  private Pose2d initialPose;

  /**
   * Constructs an instance of this class.
   * 
   * @param drivetrain  The {@link SwerveSubsystem} representing the robot
   *                    drivetrain.
   * @param translation A {@link Translation2d} instance describing the line on
   *                    which to travel.
   */
  public ProfiledDriveStraight(SwerveSubsystem drivetrain, Translation2d translation) {
    this.drivetrain = drivetrain;
    this.heading = translation.getAngle();
    this.kinematics = drivetrain.getKinematics();
    this.controller = drivetrain.createDriveController();
    this.profile = new TrapezoidProfile(
        new TrapezoidProfile.Constraints(drivetrain.getMaxSpeed(), drivetrain.getMaxAcceleration()),
        new TrapezoidProfile.State(translation.getNorm(), 0));

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    initialPose = drivetrain.getPosition();
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    // Calculate the next state (position and velocity) of motion using the
    // trapezoidal profile.
    TrapezoidProfile.State state = profile.calculate(timer.get());

    // Determine the next position on the field by offsetting the initial position
    // by the distance moved along the line of travel.
    Translation2d offset = new Translation2d(state.position, heading);
    Pose2d nextPose = initialPose.plus(new Transform2d(offset, initialPose.getRotation()));

    // Calculate the swerve drive modules states needed to reach the next state.
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
