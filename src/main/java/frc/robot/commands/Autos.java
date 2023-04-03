// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.util.FileUtil.withExtension;
import static frc.robot.util.FilesystemUtil.getPathplannerDirectory;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.stream.Collectors;

import org.javatuples.LabelValue;

import com.nrg948.autonomous.AutonomousCommandGenerator;
import com.nrg948.autonomous.AutonomousCommandMethod;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.subsystems.ShooterSubsystem.GoalShooterRPM;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FileUtil;

/**
 * A class of static methods used to create command sequences for autonomous
 * routines.
 */
public final class Autos {

  public static int MAX_GAME_PIECES_TO_SCORE = 2;
  
  private static final double FIELD_WIDTH_METERS = 8.02; // meters

  /**
   * The location of the center of the blue alliance charging station on the
   * field.
   */
  private static final Pose2d BLUE_CHARGING_STATION_CENTER = new Pose2d(4, 2.78, new Rotation2d()); // 3.89

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
  private static final double OUTER_SPEED_PERCENT = 0.8;
  private static final double COOP_SPEED_PERCENT = 0.5;

  private static final AtomicBoolean balanceOnChargingStation = new AtomicBoolean(true);
  private static final AtomicInteger numberOfGamePieces = new AtomicInteger(1);

  /**
   * Returns whether to balance on the charging station at the end of autonomous.
   * 
   * @return Whether to balance on the charging station.
   */
  public static boolean getBalanceOnChargingStation() {
    return balanceOnChargingStation.get();
  }

  /**
   * Sets whether to balance on the charging station at the end of autonomous.
   * 
   * @param balance Whether to balance on the charging station.
   */
  public static void setBalanceOnChargingStation(boolean balance) {
    balanceOnChargingStation.set(balance);
  }

  /**
   * Returns the number of game pieces to score during autonomous.
   * 
   * @return The number of game pieces to score during autonomous.
   */
  public static int getNumberOfGamePieces() {
    return numberOfGamePieces.get();
  }

  /**
   * Sets the number of game pieces to score during autonomous.
   * 
   * @param number The number of game pieces to score during autonomous.
   */
  public static void setNumberOfGamePieces(int number) {
    numberOfGamePieces.set(number);
  }

  /**
   * Returns the speed to drive during autonomous.
   * 
   * @param drivetrain The swerve drive subsystem.
   * 
   * @return The speed to drive during autonomous.
   */
  public static double getAutoSpeed(SwerveSubsystem drivetrain, boolean isOuterPath) {
    return (isOuterPath ? OUTER_SPEED_PERCENT : COOP_SPEED_PERCENT) * drivetrain.getMaxSpeed();
  }

  /**
   * Returns the acceleration to drive during autonomous.
   * 
   * @param drivetrain The swerve drive subsystem.
   * @return The acceleration to drive during autonomous.
   */
  private static double getAutoAcceleration(SwerveSubsystem drivetrain) {
    return drivetrain.getMaxAcceleration() * 0.1;
  }

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
            new Pose2d(3, 0, new Rotation2d(0))));
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
   * <p>
   * Multiple pathplanner files with the naming convention
   * &quot;&lt;_origin&gt;[-Score
   * &lt;_n&gt;]&quot; will be combined into a single sequence. The path with name
   * &quot;&lt;_origin&gt;&quot; will be followed unconditionally. The path with
   * name
   * &quot;&lt;_origin&gt;-Score &lt;_n&gt;&quot; will be selected based on the
   * number of game
   * elements to score.
   * <p>
   * Files the do not conform to the naming convention described above will be
   * treated as separate paths and unique sequences created for each.
   * <p>
   * After following the pathplanner paths, the sequence will optionally drive to
   * the center of the charging station and auto-balance.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return A {@link Collection} of {@link LabelValue} objects mapping a display
   *         name to a {@link Command} to follow a PathPlanner path group.
   */
  @AutonomousCommandGenerator
  public static Collection<LabelValue<String, Command>> getPathplannerCommands(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;

    // Generate path following commands for all pathplanner files, group them by
    // origin and then create the autonomous command sequences.
    var groupedCommands = Arrays.stream(getPathplannerDirectory().listFiles(withExtension(".path")))
        .map(FileUtil::basenameOf)
        .sorted()
        .map(n -> new LabelValue<String, Pair<Command, Pose2d>>(n, getPathplannerCommand(subsystems, n)))
        .collect(Collectors.groupingBy((lv) -> lv.getLabel().split("-Score")[0]));
    ArrayList<LabelValue<String, Command>> commandSequences = new ArrayList<>();

    groupedCommands.forEach((origin, paths) -> {
      Command sequence;
      Pose2d startPose2d = paths.get(0).getValue().getSecond();

      sequence = Commands.sequence(
          // Set the initial position of the robot.
          Commands.runOnce(() -> drivetrain.resetPosition(startPose2d), drivetrain),
          // If we're scoring at least one game piece, run the intial scoring sequence.
          Commands.either(
            Scoring.shootToTarget(subsystems, GoalShooterRPM.HIGH),
            Commands.none(),
            () -> getNumberOfGamePieces() != 0),
          // Follow the primary segment of the autonomous path.
          paths.get(0).getValue().getFirst(),
          // Follow the second segment of the autonomous path based on the number of game
          // elements to score.
          Commands.either(
              // The path at index 1 places the robot in position to auto-balance on the
              // charging station.
              paths.size() >= 2 ? paths.get(1).getValue().getFirst() : Commands.none(),
              // The path at index 2 scores a second game piece leaving the robot at the grid.
              paths.size() >= 3 ? paths.get(2).getValue().getFirst() : Commands.none(),
              () -> getNumberOfGamePieces() <= 1),
          // Drive to the center of the correct alliance charging station and auto-balance
          // if enabled. Otherwise, do nothing.
          Commands.either(
              Commands.select(
                  Map.of(
                      Alliance.Blue,
                      Commands.sequence(
                          new DriveStraight(drivetrain, BLUE_CHARGING_STATION_CENTER, getAutoSpeed(drivetrain, false)),
                          new AutoBalanceOnChargeStation(drivetrain),
                          new RainbowCycle(subsystems.leds)),
                      Alliance.Red,
                      Commands.sequence(
                          new DriveStraight(drivetrain, RED_CHARGING_STATION_CENTER, getAutoSpeed(drivetrain, false)),
                          new AutoBalanceOnChargeStation(drivetrain),
                          new RainbowCycle(subsystems.leds)),
                      Alliance.Invalid,
                      new PrintCommand("ERROR: Invalid alliance color!")),
                  DriverStation::getAlliance),
              Commands.none(),
              Autos::getBalanceOnChargingStation));
      sequence.setName(origin);

      commandSequences.add(new LabelValue<String, Command>(origin, sequence));
    });
    return commandSequences;
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
  public static Pair<Command, Pose2d> getPathplannerCommand(Subsystems subsystems, String pathGroupName) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    boolean isOuterPath = pathGroupName.contains("Outer");
    List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(
        pathGroupName,
        new PathConstraints(getAutoSpeed(drivetrain, isOuterPath), getAutoAcceleration(drivetrain)));
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
        drivetrain::getPosition,
        drivetrain::resetPosition,
        drivetrain.getKinematics(),
        new PIDConstants(1.0, 0, 0),
        new PIDConstants(1.0, 0, 0),
        drivetrain::setModuleStates,
        getPathplannerEventMap(subsystems, pathGroup),
        true,
        drivetrain);

    Command pathCommand = autoBuilder.fullAuto(pathGroup);
    Pose2d startPose2d = pathGroup.get(0).getInitialHolonomicPose();

    return new Pair<Command, Pose2d>(pathCommand, startPose2d);
  }

  /**
   * Returns the map of event marker names to commands to run when the marker is
   * reached.
   * 
   * @param subsystems The subsystems container.
   * 
   * @return The map of event marker names to commands.
   */
  private static Map<String, Command> getPathplannerEventMap(Subsystems subsystems, List<PathPlannerTrajectory> pathGroup) {
    PathPlannerState endState = pathGroup.get(0).getEndState(); 
    Pose3d endPose = new Pose3d(new Pose2d(endState.poseMeters.getTranslation(), endState.holonomicRotation));
    Pose3d aprilTagPose = endPose.transformBy(subsystems.aprilTag.getRobotToTargetTransform());
    Pose3d cubePose = endPose.transformBy(subsystems.cubeVision.getRobotToTargetTransform());

    return Map.of(
          "IntakeGamePiece", Scoring.intakeGamePiece(subsystems).withTimeout(3),
          "ScoreGamePieceMid", Scoring.shootToTarget(subsystems, GoalShooterRPM.MID).withTimeout(3),
          "ScoreGamePieceHybrid", Scoring.shootToTarget(subsystems, GoalShooterRPM.HYBRID).withTimeout(3),
          "ScoreMidFromChargeStation", Scoring.shootToTarget(subsystems,GoalShooterRPM.MID_CHARGE_STATION).withTimeout(3),
          "EnableAprilTagPoseEstimation", Commands.runOnce(() -> subsystems.drivetrain.enablePoseEstimation(subsystems.aprilTag, aprilTagPose)),
          "EnableCubePoseEstimation", Commands.runOnce(() -> subsystems.drivetrain.enablePoseEstimation(subsystems.cubeVision, cubePose)),
          "DisablePoseEstimation", Commands.runOnce(() -> subsystems.drivetrain.disablePoseEstimation())
          );
  }

  @AutonomousCommandMethod(name = "Score Cube And Drive Out Of Community")
  public static Command scoreCube(Subsystems subsystems) {
    return Commands.sequence(
      Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d(0,0,new Rotation2d(Math.PI)))),
      Commands.runEnd(() -> subsystems.intake.runMotor(-1.5), () -> subsystems.intake.stopMotor()).withTimeout(1.5),
      new DriveStraight(subsystems.drivetrain, new Translation2d(0.5,0), getAutoSpeed(subsystems.drivetrain,false), Rotation2d.fromDegrees(90)),
      new DriveStraight(subsystems.drivetrain, new Translation2d(-0.5,0), getAutoSpeed(subsystems.drivetrain, false), Rotation2d.fromDegrees(90)),
      new DriveStraight(subsystems.drivetrain, new Translation2d(3,0),getAutoSpeed(subsystems.drivetrain, false), new Rotation2d(0))
    );
  }

  @AutonomousCommandMethod(name = "Score Cube-No Drive Out Of Community")
  public static Command scoreCubeTwo(Subsystems subsystems) {
    return Commands.sequence(
      Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d(0,0,new Rotation2d(Math.PI)))),
      Commands.runEnd(() -> subsystems.intake.runMotor(-1.5), () -> subsystems.intake.stopMotor()).withTimeout(1.5),
      new DriveStraight(subsystems.drivetrain, new Translation2d(0.5,0), getAutoSpeed(subsystems.drivetrain,false), Rotation2d.fromDegrees(90)),
      new DriveStraight(subsystems.drivetrain, new Translation2d(-0.5,0), getAutoSpeed(subsystems.drivetrain, false), Rotation2d.fromDegrees(90))
    );
  }
  
  @AutonomousCommandMethod(name = "Bumper score and drive out")
  public static Command BumperScore(Subsystems subsystems) {
    return Commands.sequence(
      Commands.runOnce(() -> subsystems.drivetrain.resetPosition(new Pose2d(0,0,new Rotation2d(Math.PI)))),
      new DriveStraight(subsystems.drivetrain, new Translation2d(-0.3,0), getAutoSpeed(subsystems.drivetrain,false), Rotation2d.fromDegrees(90)),
      new DriveStraight(subsystems.drivetrain, new Translation2d(3.5,0), getAutoSpeed(subsystems.drivetrain,false), Rotation2d.fromDegrees(90))
    );
  }

  @AutonomousCommandMethod(name = "[BACKUP] Score High And Auto Balance")
  public static Command scoreAndBalance(Subsystems subsystems) {
    return Commands.sequence(
      // TODO: Set initial position
      Scoring.shootToTarget(subsystems, GoalShooterRPM.HIGH),
      Commands.waitSeconds(1),
      Commands.either(
        new DriveStraight(subsystems.drivetrain, RED_CHARGING_STATION_CENTER, getAutoSpeed(subsystems.drivetrain, false)),
        new DriveStraight(subsystems.drivetrain, BLUE_CHARGING_STATION_CENTER, getAutoSpeed(subsystems.drivetrain, false)), 
        () -> DriverStation.getAlliance().equals(Alliance.Red)),
      new AutoBalanceOnChargeStation(subsystems.drivetrain)
    );
  }
  

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
