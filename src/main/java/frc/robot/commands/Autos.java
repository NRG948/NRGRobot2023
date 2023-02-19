// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.FileUtil.withExtension;
import static frc.robot.util.FilesystemUtil.getPathplannerDirectory;

import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import org.javatuples.LabelValue;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FileUtil;

public final class Autos {
  @AutonomousCommandMethod(name = "Follow S-Curve Path")
  public static CommandBase followSCurvePath(Subsystems subsystems) {
    return Commands.sequence(
        Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        FollowTrajectory.fromWaypoints(
            subsystems.drivetrain,
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(new Translation2d(1, 1), new Translation2d(2, 1)),
            new Pose2d(3, 0,
                new Rotation2d(0))));
  }

  @AutonomousCommandMethod(name = "Drive Straight For 3 Meters")
  public static CommandBase driveStraight3Meters(Subsystems subsystems) {
    return Commands.sequence(
        Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        new DriveStraight(subsystems.drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(0))));
  }

  @AutonomousCommandMethod(name = "Drive Diagonal For 3 Meters")
  public static CommandBase driveDiagonal3Meters(Subsystems subsystems) {
    return Commands.sequence(
        Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        new DriveStraight(subsystems.drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(45.0))));
  }

  @AutonomousCommandMethod(name = "Drive Straight For 3 Meters and Rotate")
  public static CommandBase driveStraight3MetersAndRotate(Subsystems subsystems) {
    return Commands.sequence(
        Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        new DriveStraight(subsystems.drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(0)),
            subsystems.drivetrain.getMaxSpeed() * 0.5, Rotation2d.fromDegrees(-90.0)));
  }

  /**
   * Returns a {@link Collection} of {@link LabelValue} objects mapping a display
   * name to a {@link Command} to follow a path group stored in the PathPlanner
   * deployment directory.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return A {@link Collection} of {@link LabelValue} objects mapping a display
   *         name to a {@link Command} to follow a PathPlanner path group.
   */
  @AutonomousCommandGenerator
  public static Collection<LabelValue<String, Command>> getPathplannerCommands(Subsystems subsystems) {
    return Arrays.stream(getPathplannerDirectory().listFiles(withExtension(".path")))
        .map(FileUtil::basenameOf)
        .map(n -> new LabelValue<String, Command>(n, getPathplannerCommand(subsystems, n)))
        .toList();
  }

  /**
   * Returns a {@link Command} to follow a path group stored in the PathPlanner
   * deployment directory.
   * 
   * @param subsystems The subsystems container.
   * @param pathGroupName The path group to follow.
   * 
   * @return A {@link Command} to follow a PathPlanner path group.
   */
  public static Command getPathplannerCommand(Subsystems subsystems, String pathGroupName) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
        pathGroupName,
        new PathConstraints(drivetrain.getMaxSpeed(), drivetrain.getMaxAcceleration()));
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrain::getPosition,
        drivetrain::resetPosition,
        drivetrain.getKinematics(),
        new PIDConstants(1.0, 0, 0),
        new PIDConstants(1.0, 0, 0),
        drivetrain::setModuleStates,
        Map.of(),
        true,
        drivetrain);

    return autoBuilder.fullAuto(pathGroup);
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
