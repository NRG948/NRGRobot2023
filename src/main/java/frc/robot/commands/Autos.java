// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.File;
import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Map;

import org.javatuples.LabelValue;

import com.nrg948.autonomous.AutonomousCommandMethod;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

public final class Autos {
  @AutonomousCommandMethod(name = "Follow S-Curve Path")
  public static CommandBase followSCurvePath(Subsystems subsystems) {
    return Commands.sequence(
        new InstantCommand(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
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
        new InstantCommand(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        new ProfiledDriveStraight(subsystems.drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(0))));
  }

  @AutonomousCommandMethod(name = "Drive Diagonal For 3 Meters")
  public static CommandBase driveDiagonal3Meters(Subsystems subsystems) {
    return Commands.sequence(
        new InstantCommand(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        new ProfiledDriveStraight(subsystems.drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(45.0))));
  }

  @AutonomousCommandMethod(name = "Drive Straight For 3 Meters and Rotate")
  public static CommandBase driveStraight3MetersAndRotate(Subsystems subsystems) {
    return Commands.sequence(
        new InstantCommand(() -> subsystems.drivetrain.resetPosition(new Pose2d())),
        new ProfiledDriveStraight(subsystems.drivetrain, new Translation2d(3.0, Rotation2d.fromDegrees(0)),
            subsystems.drivetrain.getMaxSpeed() * 0.5, Rotation2d.fromDegrees(-90.0)));
  }

  /**
   * Returns a command sequence that drives the robot to the grid based on the
   * detected AprilTag and selected alignment (left, center or right), and scores
   * the acquired game element.
   * <p>
   * The manipulator selects a grid scoring position from the Xbox controller's
   * d-pad. When the d-pad is not pressed, it selects the center middle of the
   * grid. Pressing the d-pad diagonally up, side and diagonally down select the
   * left and right high, middle and lower scoring grids, respectively. The
   * selection is indicated on the Operator tab of Shuffleboard. When a selection
   * is made, the manipulator presses and holds the right bumper to execute the
   * scoring sequence.
   * <p>
   * TODO: Finalize the scoring sequence selection interface with the manipulator.
   * 
   * @param subsystems            The subsystems container.
   * @param manipulatorController The manipulator's Xbox controller.
   * 
   * @return A command sequence to drive the robot to the grid and score.
   */
  public static Command scoreToGrid(Subsystems subsystems, XboxController manipulatorController) {
    var drivetrain = subsystems.drivetrain;

    var target = subsystems.photonVision.getBestTarget();
    var cameraToTarget = target.getBestCameraToTarget();

    // Transform the robot's pose to find the tag's pose
    var robotPose = drivetrain.getPosition3d();
    var cameraPose = robotPose.transformBy(RobotConstants.ROBOT_TO_CAMERA);
    System.out.println("CAMERA POSE = " + cameraPose);
    var targetPose = cameraPose.transformBy(cameraToTarget);
    System.out.println("TARGET POSE = " + targetPose);

    // Determine the grid aligment y-offset for scoring on left, center or right.
    double yOffset = 0;

    int pov = manipulatorController.getPOV();
    if (pov >= 45 && pov <= 145) {
      // Left side
      yOffset = -RobotConstants.GRID_SIDE_OFFSET;
    } else if (pov >= 225 && pov <= 315) {
      // Right side
      yOffset = RobotConstants.GRID_SIDE_OFFSET;
    }

    // Find the scoring position pose.
    var tagToGoal = new Transform3d(
        new Translation3d(RobotConstants.SCORING_DISTANCE_FROM_GRID + (RobotConstants.ROBOT_LENGTH / 2), yOffset, 0.0),
        new Rotation3d(0, 0, Math.PI));
    var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

    System.out.println("GOAL POSE = " + goalPose);

    var driveVector = goalPose.relativeTo(robotPose.toPose2d()).getTranslation();

    return Commands.sequence(
        new ProfiledDriveStraight(drivetrain, driveVector, drivetrain.getMaxSpeed() * 0.5, new Rotation2d(0))
    // TODO: Add scoring sequence here
    );
  }

  /**
   * Returns a collection of pathfinder commands.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return Collection of pathfinder commands.
   */
  public static Collection<LabelValue<String, Command>> getPathfinderCommands(Subsystems subsystems) {
    File deployDir = Filesystem.getDeployDirectory();
    File pathfinderDir = new File(deployDir, "pathplanner");
    ArrayList<LabelValue<String, Command>> commands = new ArrayList<>();
    File[] files = pathfinderDir.listFiles((dir, fileName) -> fileName.endsWith(".path"));
    for (File file : files) {
      String fileName = file.getName();
      SwerveSubsystem drivetrain = subsystems.drivetrain;
      String pathName = fileName.substring(0, fileName.lastIndexOf("."));
      List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
          pathName,
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
      commands.add(new LabelValue<String, Command>(pathName, autoBuilder.fullAuto(pathGroup)));
    }

    return commands;
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
