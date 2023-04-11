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
import java.util.function.BooleanSupplier;
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
import frc.robot.subsystems.AdressableLEDSubsystem;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.subsystems.CubeVisionSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystemBase;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.FileUtil;

/**
 * A class of static methods used to create command sequences for autonomous
 * routines.
 */
public final class Autos {
  private static final Rotation2d ORIENT_ZERO_DEGREES = new Rotation2d(0);
  private static final Rotation2d ORIENT_90_DEGREES = Rotation2d.fromDegrees(90);

  public static final int MAX_GAME_PIECES_TO_SCORE = 2;

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
  private static final double COOP_SPEED_PERCENT = 0.55;

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
            new Pose2d(0, 0, ORIENT_ZERO_DEGREES),
            List.of(new Translation2d(1, 1), new Translation2d(2, 1)),
            new Pose2d(3, 0, ORIENT_ZERO_DEGREES)));
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
                      Alliance.Blue, driveAndBalance(subsystems, BLUE_CHARGING_STATION_CENTER),
                      Alliance.Red, driveAndBalance(subsystems, RED_CHARGING_STATION_CENTER),
                      Alliance.Invalid, new PrintCommand("ERROR: Invalid alliance color!")),
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
    boolean isOuterPath = pathGroupName.contains("Outer") && !pathGroupName.contains("Wall");
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
        getPathplannerEventMap(subsystems, pathGroupName, pathGroup),
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
   * @param pathGroupName The path group name.
   * @param pathGroup The Pathplanner path group.
   * 
   * @return The map of event marker names to commands.
   */
  private static Map<String, Command> getPathplannerEventMap(
      Subsystems subsystems,
      String pathGroupName,
      List<PathPlannerTrajectory> pathGroup) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    AprilTagSubsystem aprilTag = subsystems.aprilTag;
    CubeVisionSubsystem cubeVision = subsystems.cubeVision;

    Command enableAprilTagPoseEstimation = Commands.none();
    Command enableCubePoseEstimation = Commands.none();

    if (pathGroupName.contains("With Vision") && pathGroupName.endsWith("-Score 2")) {
      // The pose of the cube can be determined from the robot pose at the end of the first path segment.
      Pose3d cubePose = getRobotPose3d(pathGroup.get(0))
          .transformBy(cubeVision.getRobotToTargetTransform());

      enableCubePoseEstimation = estimatePose(drivetrain, cubeVision, cubePose);

      // The pose of the AprilTag can be determined from the robot pose at the end of the second path segment.
      if (pathGroup.size() >= 2) {
        Pose3d aprilTagPose = getRobotPose3d(pathGroup.get(1))
            .transformBy(aprilTag.getRobotToTargetTransform());

        enableAprilTagPoseEstimation = estimatePose(drivetrain, aprilTag, aprilTagPose);
      }
    }

    return Map.of(
        "IntakeGamePiece", Scoring.intakeGamePiece(subsystems).withTimeout(3),
        "WaitForGamePiece", Commands.waitUntil(subsystems.indexer::isCubeDetected).withTimeout(1),
        "ScoreGamePieceMid", Scoring.shootToTarget(subsystems, GoalShooterRPM.MID).withTimeout(3),
        "ScoreGamePieceHybrid", Scoring.shootToTarget(subsystems, GoalShooterRPM.HYBRID).withTimeout(3),
        "ScoreMidFromChargeStation", Scoring.shootToTarget(subsystems, GoalShooterRPM.MID_CHARGE_STATION).withTimeout(3),
        "EnableAprilTagPoseEstimation", enableAprilTagPoseEstimation.withTimeout(3),
        "EnableCubePoseEstimation", enableCubePoseEstimation.withTimeout(3),
        "DisablePoseEstimation", Commands.runOnce(() -> drivetrain.disablePoseEstimation()),
        "SetMidRPM", Commands.runOnce(() -> subsystems.shooter.setGoalRPM(GoalShooterRPM.MID)));
  }

  public static Pose3d getRobotPose3d(PathPlannerTrajectory path) {
    PathPlannerState endState = path.getEndState();

    return new Pose3d(new Pose2d(endState.poseMeters.getTranslation(), endState.holonomicRotation));
  }

  /**
   * Returns a command that enables vision-based pose estimation.
   * <p>
   * This command disables pose estimation if it previously saw a target but no
   * longer does.
   * 
   * @param drivetrain   The swerve subsystem.
   * @param visionSource The source PhotonVision subsystem.
   * @param targetPose   The expected pose of the target on the field.
   * 
   * @return A command that enables vision-based pose estimation.
   */
  public static Command estimatePose(
      SwerveSubsystem drivetrain,
      PhotonVisionSubsystemBase visionSource,
      Pose3d targetPose) {
    // Require only the PhotonVision subsystem since we do not want to interrupt
    // the current path following command running on the swerve subsystem.
    return Commands.startEnd(
        () -> drivetrain.enablePoseEstimation(visionSource, targetPose),
        () -> drivetrain.disablePoseEstimation(),
        visionSource)
        .until(new BooleanSupplier() {
          private boolean hadTargets = visionSource.hasTargets();

          @Override
          public boolean getAsBoolean() {
            boolean hasTargets = visionSource.hasTargets();

            if (hadTargets && !hasTargets) {
              return true;
            }

            hadTargets = hasTargets;

            return false;
          }
        });
  }

  /**
   * Returns a command to drive onto the center of the charging station and
   * auto-balance.
   * 
   * @param subsystems          The subsystems container.
   * @param chargingStationPose The pose at the center of the charging station.
   * 
   * @return A command to drive onto the center of the charging station and
   *         auto-balance.
   */
  public static Command driveAndBalance(Subsystems subsystems, Pose2d chargingStationPose) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    AdressableLEDSubsystem leds = subsystems.leds;
    double autoSpeed = getAutoSpeed(drivetrain, false);

    return Commands.sequence(
        new DriveStraight(drivetrain, chargingStationPose, autoSpeed),
        new AutoBalanceOnChargeStation(drivetrain),
        new RainbowCycle(leds));
  }

  @AutonomousCommandMethod(name = "Score Cube And Drive Out Of Community")
  public static Command scoreCube(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    IntakeSubsystem intake = subsystems.intake;
    double autoSpeed = getAutoSpeed(drivetrain, false);

    return Commands.sequence(
        Commands.runOnce(() -> drivetrain.resetPosition(new Pose2d(0, 0, new Rotation2d(Math.PI)))),
        Commands.runEnd(() -> intake.runMotor(-1.5), () -> intake.stopMotor()).withTimeout(1.5),
        new DriveStraight(drivetrain, new Translation2d(0.5, 0), autoSpeed, ORIENT_90_DEGREES),
        new DriveStraight(drivetrain, new Translation2d(-0.5, 0), autoSpeed, ORIENT_90_DEGREES),
        new DriveStraight(drivetrain, new Translation2d(3, 0), autoSpeed, ORIENT_ZERO_DEGREES));
  }

  @AutonomousCommandMethod(name = "Score Cube-No Drive Out Of Community")
  public static Command scoreCubeTwo(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    IntakeSubsystem intake = subsystems.intake;
    double autoSpeed = getAutoSpeed(drivetrain, false);

    return Commands.sequence(
        Commands.runOnce(() -> drivetrain.resetPosition(new Pose2d(0, 0, new Rotation2d(Math.PI)))),
        Commands.runEnd(() -> intake.runMotor(-1.5), () -> intake.stopMotor()).withTimeout(1.5),
        new DriveStraight(drivetrain, new Translation2d(0.5, 0), autoSpeed, ORIENT_90_DEGREES),
        new DriveStraight(drivetrain, new Translation2d(-0.5, 0), autoSpeed, ORIENT_90_DEGREES));
  }

  @AutonomousCommandMethod(name = "Bumper score and drive out")
  public static Command BumperScore(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    double autoSpeed = getAutoSpeed(drivetrain, false);

    return Commands.sequence(
        Commands.runOnce(() -> drivetrain.resetPosition(new Pose2d(0, 0, new Rotation2d(Math.PI)))),
        new DriveStraight(drivetrain, new Translation2d(-0.3, 0), autoSpeed, ORIENT_90_DEGREES),
        new DriveStraight(drivetrain, new Translation2d(3.5, 0), autoSpeed, ORIENT_90_DEGREES));
  }

  @AutonomousCommandMethod(name = "[BACKUP] Score High And Auto Balance")
  public static Command scoreAndBalance(Subsystems subsystems) {
    SwerveSubsystem drivetrain = subsystems.drivetrain;
    double autoSpeed = getAutoSpeed(drivetrain, false);

    return Commands.sequence(
        // TODO: Set initial position
        Scoring.shootToTarget(subsystems, GoalShooterRPM.HIGH),
        Commands.waitSeconds(1),
        Commands.either(
            new DriveStraight(drivetrain, RED_CHARGING_STATION_CENTER, autoSpeed),
            new DriveStraight(drivetrain, BLUE_CHARGING_STATION_CENTER, autoSpeed),
            () -> DriverStation.getAlliance().equals(Alliance.Red)),
        new AutoBalanceOnChargeStation(drivetrain));
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
