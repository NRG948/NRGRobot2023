// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;
import java.util.Map;

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
import edu.wpi.first.math.util.Units;
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

  @AutonomousCommandMethod(name = "Test Path")
  public static CommandBase followTestPath(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
        "Test",
        new PathConstraints(drivetrain.getMaxSpeed() * 0.5, drivetrain.getMaxAcceleration()));
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

    return autoBuilder.fullAuto(pathGroup).andThen(new AutoBalanceOnChargeStation2(drivetrain));
  }

  /**
   * Returns a command sequence that drives the robot to the grid based on the
   * detected AprilTag and selected alignment (left, center or right), and scores
   * the acquired game element.
   * 
   * @param subsystems            The subsystems container.
   * @param manipulatorController The manipulator's Xbox controller.
   * 
   * @return A command sequence to drive the robot to the grid and score.
   */
  public static Command scoreToGrid(Subsystems subsystems, XboxController manipulatorController) {
    var target = subsystems.photonVision.getBestTarget();
    var cameraToTarget = target.getBestCameraToTarget();
    var robotPose = subsystems.drivetrain.getPosition3d();

    // Transform the robot's pose to find the tag's pose
    var cameraPose = robotPose.transformBy(RobotConstants.CAMERA_TO_ROBOT);
    System.out.println("CAMERA POSE = " + cameraPose);
    var targetPose = cameraPose.transformBy(cameraToTarget);
    System.out.println("TARGET POSE = " + targetPose);

    // Determine the grid aligment y-offset for scoring on left, center or right.
    double yOffset = 0;

    int pov = manipulatorController.getPOV();
    if (pov >= 45 && pov <= 145) {
      yOffset = Units.inchesToMeters(22);
    } else if (pov >= 225 && pov <= 315) {
      yOffset = Units.inchesToMeters(-22);
    }

    // Find the scoring position pose.
    var tagToGoal = new Transform3d(
        new Translation3d(Units.inchesToMeters(15), yOffset, 0.0),
        new Rotation3d(0, 0, Math.PI));
    var goalPose = targetPose.transformBy(tagToGoal).toPose2d();

    System.out.println("GOAL POSE = " + goalPose);

    Translation2d translationToGoal = goalPose.getTranslation();
    Translation2d driveVector = new Translation2d(translationToGoal.getNorm(), translationToGoal.getAngle());

    return Commands.sequence(
        new ProfiledDriveStraight(subsystems.drivetrain, driveVector, subsystems.drivetrain.getMaxSpeed() * 0.5,
            new Rotation2d(0))
    // TODO: Add scoring sequence here
    );
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
