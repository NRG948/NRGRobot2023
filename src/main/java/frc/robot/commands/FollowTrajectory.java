// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.drive.SwerveModule;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A command to follow a trajectory.
 */
public class FollowTrajectory extends SwerveControllerCommand {
  private final SwerveSubsystem drivetrain;

  /**
   * Returns a FollowTrajectory command to follow the path defined by the
   * wavepoints.
   * 
   * @param drivetrain  A SwerveSubsystem object.
   * @param start       The start of the path.
   * @param waypoints   A list of intermediate waypoints.
   * @param end         The end of the path.
   * @param constraints Constraints to enforce along the path.
   * @return A FollowTrajectory command.
   */
  public static FollowTrajectory fromWaypoints(
      SwerveSubsystem drivetrain,
      Pose2d start,
      List<Translation2d> waypoints,
      Pose2d end,
      TrajectoryConstraint... constraints) {
    TrajectoryConfig config = new TrajectoryConfig(SwerveModule.kMaxDriveSpeed, SwerveModule.kMaxDriveAcceleration)
        .addConstraint(new SwerveDriveKinematicsConstraint(SwerveSubsystem.kKinematics, SwerveModule.kMaxDriveSpeed))
        .addConstraints(List.of(constraints));
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(start, waypoints, end, config);

    return new FollowTrajectory(drivetrain, trajectory);
  }

  /**
   * Returns a FollowTrajectory command to follow a path defined by a Pathweaver
   * Json file.
   * 
   * @param drivetrain A SwerveSubsystem object.
   * @param path       The path from Pathweaver Json file.
   * @return A FollowTrajectory command.
   * @throws IOException
   */
  public static FollowTrajectory fromPathweaverJson(SwerveSubsystem drivetrain, Path path) throws IOException {
    Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(path);

    return new FollowTrajectory(drivetrain, trajectory);
  }

  /**
   * Constructs an instance of this class.
   * 
   * @param drivetrain A SwerveSubsystem object.
   * @param trajectory The path to follow.
   */
  public FollowTrajectory(SwerveSubsystem drivetrain, Trajectory trajectory) {
    super(
        trajectory,
        drivetrain::getPosition,
        SwerveSubsystem.kKinematics,
        new HolonomicDriveController(
            new PIDController(1.0, 0.0, 0.0),
            new PIDController(1.0, 0.0, 0.0),
            new ProfiledPIDController(1.0, 0.0, 0.0, SwerveModule.kSteeringConstraints)),
        drivetrain::setModuleStates);
    this.drivetrain = drivetrain;
  }

  @Override
  public void end(boolean interrupted) {
    super.end(interrupted);
    drivetrain.stopMotors();
  }
}
