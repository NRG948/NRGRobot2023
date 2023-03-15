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
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClawSubsystem.Position;
import frc.robot.subsystems.ElevatorAngleSubsystem;
import frc.robot.subsystems.ElevatorAngleSubsystem.ElevatorAngle;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystem.GoalState;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;

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

    // Determine the grid scoring level.
    GoalState targetState = GoalState.SCORE_MID;
    if (pov != -1) {
      if (pov <= 45 || pov >= 315) {
        targetState = GoalState.SCORE_LOW;
      } else if (pov >= 135 && pov <= 225) {
        targetState = GoalState.SCORE_HIGH;
      }
    }

    return Commands.sequence(
        new DriveStraight(drivetrain, goalPose, drivetrain.getMaxSpeed() * 0.5),
        scoreGamePiece(subsystems, targetState),
        prepareToAcquire(subsystems));
  }

  /**
   * Returns commands sequence to score the game piece to the desired level.
   * 
   * @param subsystems The subsystems container.
   * @param target     The target scoring level.
   * 
   * @return A command sequence to automate scoring the game piece.
   */
  public static Command scoreGamePiece(Subsystems subsystems, GoalState target) {
    ClawSubsystem claw = subsystems.claw;
    ElevatorSubsystem elevator = subsystems.elevator;
    ElevatorAngleSubsystem elevatorAngle = subsystems.elevatorAngle;

    return Commands.sequence(
        // Set the elevator angle to scoring position and begins raising the elevator to
        // at least the low scoring position so the claw arm will flip over but clear
        // the upper crossbar.
        Commands.runOnce(() -> elevator.setGoal(GoalState.FLIP), elevator),
        Commands.waitUntil(elevator::atGoal),
        Commands.runOnce(() -> elevatorAngle.setGoalAngle(ElevatorAngle.SCORING), elevatorAngle),
        Commands.waitUntil(elevatorAngle::atGoalAngle),
        // Raise the elevator to the desired scoring elevation.
        Commands.runOnce(() -> elevator.setGoal(target), elevator),
        Commands.waitUntil(elevator::atGoal)
       );
  }

  /**
   * Set the elevator and claw to acquiring position.
   * 
   * @param subsystems The Subsytems container.
   * @return A command sequence to set the elevator and claw to acquiring
   *         position.
   */
  public static Command prepareToAcquire(Subsystems subsystems) {
    ClawSubsystem claw = subsystems.claw;
    ElevatorSubsystem elevator = subsystems.elevator;
    ElevatorAngleSubsystem elevatorAngle = subsystems.elevatorAngle;

    return Commands.sequence(
        // If the elevator is currently at the high scoring position, lower it to the
        // middle scoring position so that the claw arm can clear the upper crossbar
        // when it flips over to the acquiring position.
        Commands.either(
            Commands.runOnce(() -> elevator.setGoal(GoalState.FLIP), elevator)
                .andThen(Commands.waitUntil(elevator::atGoal)),
            Commands.none(),
            () -> !elevator.atPosition(GoalState.SCORE_LOW)),
        // Set the elevator angle to aquiring and set the elevator carriage to low
        // scoring position to avoid coliding with the intake as the claw flips over.
        
            Commands.runOnce(() -> elevatorAngle.setGoalAngle(ElevatorAngle.ACQUIRING), elevatorAngle),
            Commands.waitUntil(elevatorAngle::atGoalAngle),
        // Close the claw and lower the carriage to aquiring position.
        Commands.runOnce(() -> elevator.setGoal(GoalState.ACQUIRE), elevator),
        Commands.waitUntil(elevator::atGoal),
        Commands.runOnce(() -> claw.set(Position.CLOSED), claw));
  }

  /**
   * Returns a command to prep the robot for a match.
   * 
   * @param subsystems The subsystems container.
   * @return A command to prep the robot for a match.
   */
  public static Command prepForMatch(Subsystems subsystems) {
    ClawSubsystem claw = subsystems.claw;
    ElevatorSubsystem elevator = subsystems.elevator;
    ElevatorAngleSubsystem elevatorAngle = subsystems.elevatorAngle;
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    SwerveModuleState intialState = new SwerveModuleState();

    return Commands.sequence(
        Commands.runEnd(() -> elevatorAngle.setMotorVoltage(-1), () -> elevatorAngle.stopMotor(), elevatorAngle)
            .until(elevatorAngle::atAcquiringLimit),
        Commands.runEnd(() -> elevator.setMotorVoltage(-1), () -> elevator.stopMotor(), elevator)
            .until(elevator::atBottomLimit),
        Commands.runOnce(() -> claw.set(Position.CLOSED)),
        Commands.runOnce(() -> drivetrain.setModuleStates(new SwerveModuleState[] {
            intialState, intialState, intialState, intialState
        }), drivetrain));
  }

  private Scoring() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
