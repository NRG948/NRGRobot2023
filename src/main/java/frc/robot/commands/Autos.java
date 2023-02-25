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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FileUtil;

/**
 * A class of static methods used to create command sequences for autonomous
 * routines.
 */
public final class Autos {

  private static final double FIELD_WIDTH_METERS = 8.02; // meters

  /**
   * The location of the center of the blue alliance charging station on the
   * field.
   */
  private static final Pose2d BLUE_CHARGING_STATION_CENTER = new Pose2d(3.83, 2.73, new Rotation2d());

  /**
   * The location of the center of the red alliance charging station on the
   * field.
   * <p>
   * The pose is a mirror of the blue alliance charging station pose.
   */
  private static final Pose2d RED_CHARGING_STATION_CENTER = new Pose2d(
      BLUE_CHARGING_STATION_CENTER.getX(),
      FIELD_WIDTH_METERS - BLUE_CHARGING_STATION_CENTER.getY(),
      BLUE_CHARGING_STATION_CENTER.getRotation().times(-1));

  /**
   * The speed of driving during autonomous as a percent of the maximum robot
   * speed. We must use a value lower than maximum to allow headroom for course
   * corrections.
   */
  private static final double AUTO_SPEED_PERCENT = 0.8;

  private static Map<String, Command> pathplannerEventMap;

  /**
   * Creates a command sequence to follow an S-curve path. It sets the initial
   * position of the robot to (0, 0, 0°).
   * <p>
   * This command is used to test the WPILib trajectory following integration with
   * the robot.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return A command sequence to follow an S-curve path.
   */
  @AutonomousCommandMethod(name = "[TEST] Follow S-Curve Path")
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

  /**
   * Creates a command sequence to drive to robot straight forward for 3 meters.
   * It sets the initial position of the robot to (0, 0, 0°).
   * <p>
   * This command is used to test the {@link DriveStraight} command.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return A command sequence to drive the robot straight forward for 3 meters.
   */
  @AutonomousCommandMethod(name = "[TEST] Drive Straight For 3 Meters")
  public static CommandBase driveStraight3Meters(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;

    return Commands.sequence(
        Commands.runOnce(() -> drivetrain.resetPosition(new Pose2d())),
        new DriveStraight(drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(0)),
            drivetrain.getMaxSpeed() * 0.667));
  }

  /**
   * Creates a command sequence to drive to robot diagonally for 3 meters at a
   * heading of 45°. It sets the initial position of the robot to (0, 0, 0°).
   * <p>
   * This command is used to test the {@link DriveStraight} command.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return A command sequence to drive the robot straight forward for 3 meters.
   */
  @AutonomousCommandMethod(name = "[TEST] Drive Diagonal For 3 Meters")
  public static CommandBase driveDiagonal3Meters(Subsystems subsystems) {
    return Commands.sequence(
        Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        new DriveStraight(subsystems.drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(45.0))));
  }

  /**
   * Creates a command sequence to drive to robot straight forward for 3 meters
   * while rotating to an orientation of -90°. It sets the initial position of the
   * robot to (0, 0, 0°).
   * <p>
   * This command is used to test the {@link DriveStraight} command.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return A command sequence to drive the robot straight forward for 3 meters
   *         while rotating to an orientation of -90°.
   */
  @AutonomousCommandMethod(name = "[TEST] Drive Straight For 3 Meters and Rotate")
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
   * @param subsystems    The subsystems container.
   * @param pathGroupName The path group to follow.
   * 
   * @return A {@link Command} to follow a PathPlanner path group.
   */
  public static Command getPathplannerCommand(Subsystems subsystems, String pathGroupName) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
        pathGroupName,
        new PathConstraints(drivetrain.getMaxSpeed() * AUTO_SPEED_PERCENT, drivetrain.getMaxAcceleration()));
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrain::getPosition,
        drivetrain::resetPosition,
        drivetrain.getKinematics(),
        new PIDConstants(1.0, 0, 0),
        new PIDConstants(1.0, 0, 0),
        drivetrain::setModuleStates,
        getPathplannerEventMap(subsystems),
        true,
        drivetrain);

    return Commands.sequence(
        // Create the pathplanner command to follow the trajectory.
        autoBuilder.fullAuto(pathGroup),
        // Drive to the center of the correct alliance charging station and auto-balance.
        Commands.select(
          Map.of(
            Alliance.Blue,
            Commands.sequence(
              new DriveStraight(drivetrain, BLUE_CHARGING_STATION_CENTER, drivetrain.getMaxSpeed() * AUTO_SPEED_PERCENT),
              new AutoBalanceOnChargeStation(drivetrain)),
            Alliance.Red,
            Commands.sequence(
              new DriveStraight(drivetrain, RED_CHARGING_STATION_CENTER, drivetrain.getMaxSpeed() * AUTO_SPEED_PERCENT),
              new AutoBalanceOnChargeStation(drivetrain)),
            Alliance.Invalid,
            new PrintCommand("ERROR: Invalid alliance color!")),
          DriverStation::getAlliance));
  }

  /**
   * Returns the map of event marker names to commands to run when the marker is
   * reached.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return The map of event marker names to commands.
   */
  private static Map<String, Command> getPathplannerEventMap(Subsystems subsystems) {
    if (pathplannerEventMap == null) {
      pathplannerEventMap = Map.of();
    }
    return pathplannerEventMap;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
