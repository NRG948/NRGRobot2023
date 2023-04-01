// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.ShooterSubsystem.GoalShooterRPM;

/**
 * A class of static methods used to create command sequences to score game
 * elements.
 */
public final class Scoring {

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

    var target = subsystems.cubeVision.getBestTarget();
    var cameraToTarget = target.getBestCameraToTarget();

    // Transform the robot's pose to find the tag's pose
    var robotPose = drivetrain.getPosition3d();
    var cameraPose = robotPose.transformBy(RobotConstants.ROBOT_TO_BACK_CAMERA);
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

    // // Determine the grid scoring level.
    // GoalState targetState = GoalState.SCORE_MID;
    // if (pov != -1) {
    // if (pov <= 45 || pov >= 315) {
    // targetState = GoalState.SCORE_LOW;
    // } else if (pov >= 135 && pov <= 225) {
    // targetState = GoalState.SCORE_HIGH;
    // }
    // }

    return Commands.sequence(
        new DriveStraight(drivetrain, goalPose, drivetrain.getMaxSpeed() * 0.5));
  }

  /**
   * Returns a command sequence to intake a game piece.
   * 
   * @param subsystems The susystems container.
   * @return A command sequence to intake a game piece.
   */
  public static Command intakeGamePiece(Subsystems subsystems) {
    IntakeSubsystem intake = subsystems.intake;
    IndexerSubsystem indexer = subsystems.indexer;

    return Commands.sequence(
        Commands.parallel(
            Commands.runOnce(() -> indexer.setIntakeRPM(),
                indexer),
            Commands.runOnce(() -> intake.enable(),
                intake)),
        Commands.waitUntil(() -> indexer.isCubeDetected()),
        Commands.waitSeconds(0.1)
        ).finallyDo((interrupted) -> {
          intake.disable();
          indexer.disable();
        });
  }

  /**
   * Returns a command to shoot the cube into the specified target.
   * 
   * @param subsystems The subsystems container.
   * @param target     The target
   * @return A command to shoot the cube into the specified target
   */
  public static Command shootToTarget(Subsystems subsystems, ShooterSubsystem.GoalShooterRPM target) {
    ShooterSubsystem shooter = subsystems.shooter;
    IndexerSubsystem indexer = subsystems.indexer;
    return Commands.parallel(
        Commands.sequence(
            Commands.runOnce(() -> shooter.enable(target),
                shooter),
            Commands.waitUntil(() -> !indexer.isCubeDetected()),
            Commands.waitSeconds(0.75),
            Commands.runOnce(() -> shooter.setGoalRPM(GoalShooterRPM.HYBRID), shooter)),
        Commands.sequence(
            Commands.waitSeconds(0.75),
            Commands.runOnce(() -> indexer.setShootRPM(), indexer),
            Commands.waitUntil(() -> !indexer.isCubeDetected()),
            Commands.runOnce(() -> indexer.disable(), indexer)));
  }

  /**
   * Returns a command to shoot the cube into the specified target.
   * 
   * @param subsystems The subsystems container.
   * @param target     The target
   * @return A command to shoot the cube into the specified target
   */
  public static Command manualShootToTarget(Subsystems subsystems, ShooterSubsystem.GoalShooterRPM target) {
    ShooterSubsystem shooter = subsystems.shooter;
    IndexerSubsystem indexer = subsystems.indexer;
    return Commands.parallel(
        Commands.startEnd(() -> shooter.enable(target),
            () -> shooter.setGoalRPM(GoalShooterRPM.HYBRID),
            shooter),
        Commands.sequence(
            Commands.either(
              Commands.none(),
              Commands.waitSeconds(0.75),
              () -> shooter.getCurrentGoalRPM().equals(GoalShooterRPM.HYBRID)),
            Commands.startEnd(() -> indexer.setShootRPM(),
                () -> indexer.disable(),
                indexer)));
  }

  /**
   * Returns a command to prep the robot for a match.
   * 
   * @param subsystems The subsystems container.
   * @return A command to prep the robot for a match.
   */
  public static Command prepForMatch(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    SwerveModuleState intialState = new SwerveModuleState();

    return Commands.sequence(
        Commands.runOnce(() -> drivetrain.setModuleStates(new SwerveModuleState[] {
            intialState, intialState, intialState, intialState
        }), drivetrain));
  }

  private Scoring() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
