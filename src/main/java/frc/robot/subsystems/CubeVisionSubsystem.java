// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import org.photonvision.targeting.PhotonTrackedTarget;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanTopic;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import frc.robot.Constants.RobotConstants;

/**
 * This subsystem is responsible for getting target information from
 * PhotonVision.
 */
@RobotPreferencesLayout(groupName = "CubeVision", row = 0, column = 4, width = 2, height = 1)
public class CubeVisionSubsystem extends PhotonVisionSubsystemBase {
  private static final Pose2d TEST_ROBOT_POSE = new Pose2d();
  private static final Pose3d TEST_TARGET_POSE = new Pose3d(new Translation3d(1, 0, 0), new Rotation3d());

  private static final Transform3d TARGET_TO_ROBOT = new Transform3d(
      new Translation3d(Units.inchesToMeters(0), 0, 0),
      new Rotation3d());

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue(
      "CubeVision", "Enable Tab", false);

  private DoubleLogEntry deltaXLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Delta X");
  private DoubleLogEntry deltaYLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Delta Y");
  private DoubleLogEntry targetXLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Target X");
  private DoubleLogEntry targetYLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Target Y");
  private DoubleLogEntry targetAngleLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Target Angle");

  private Optional<Pose2d> estimatedPose = Optional.empty();
  private AtomicBoolean estimateForShuffleboard = new AtomicBoolean(false);

  /** Creates a new PhotonVisionSubsystem. */
  public CubeVisionSubsystem() {
    super("Front", RobotConstants.FRONT_CAMERA_TO_ROBOT, TARGET_TO_ROBOT);
  }

  /**
   * Adds a tab for CubeVision in Shuffleboard.
   */
  public void addShuffleboardTab() {
    if (!enableTab.getValue()) {
      return;
    }

    ShuffleboardTab visionTab = Shuffleboard.getTab("CubeVision");
    ShuffleboardLayout targetLayout = visionTab.getLayout("Target Info", BuiltInLayouts.kGrid)
        .withPosition(0, 0)
        .withSize(2, 3)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
    targetLayout.addBoolean("Has Target", this::hasTargets)
        .withPosition(0, 0);
    targetLayout.addDouble("Distance", this::getDistanceToBestTarget)
        .withPosition(0, 1);
    targetLayout.addDouble("Angle", this::getAngleToBestTarget)
        .withPosition(0, 2);

    VideoSource video = new HttpCamera("photonvision_Port_1183_MJPEG_Server", "http://10.9.48.11:1183/?action=stream",
        HttpCameraKind.kMJPGStreamer);
    visionTab.add("CubeVision", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);

    ShuffleboardLayout poseEstimateLayout = visionTab.getLayout("Pose Estimate", BuiltInLayouts.kGrid)
        .withPosition(6, 0)
        .withSize(2, 3)
        .withProperties(Map.of("Number of columns", 1, "Number of rows", 4));
    SimpleWidget poseEstimateEnabledWidget = poseEstimateLayout
        .add("Enable", estimateForShuffleboard.get())
        .withWidget(BuiltInWidgets.kToggleSwitch)
        .withPosition(0, 0);
    poseEstimateLayout.addDouble("X", () -> estimatedPose.isPresent() ? estimatedPose.get().getX() : 0)
        .withPosition(0, 1);
    poseEstimateLayout.addDouble("Y", () -> estimatedPose.isPresent() ? estimatedPose.get().getY() : 0)
        .withPosition(0, 2);
    poseEstimateLayout
        .addDouble("Angle", () -> estimatedPose.isPresent() ? estimatedPose.get().getRotation().getDegrees() : 0)
        .withPosition(0, 3);

    BooleanTopic poseEstimateEnabledTopic = new BooleanTopic(poseEstimateEnabledWidget.getEntry().getTopic());
    NetworkTableInstance.getDefault().addListener(
        poseEstimateEnabledTopic,
        EnumSet.of(NetworkTableEvent.Kind.kValueAll, NetworkTableEvent.Kind.kImmediate),
        (event) -> estimateForShuffleboard.set(event.valueData.value.getBoolean()));
  }

  private Optional<Pose2d> getEstimatedRobotPose(Pose2d robotPose, Pose3d targetPose) {
    double deltaX = (RobotConstants.FRONT_CAMERA_Z / Math.tan(Math.toRadians(90) + (RobotConstants.FRONT_CAMERA_PITCH + getPitchToBestTarget()))) + RobotConstants.FRONT_CAMERA_X;

    if (deltaX <= 0) {
      return Optional.empty();
    }

    double deltaY = deltaX * Math.tan(-getAngleToBestTarget()) + RobotConstants.FRONT_CAMERA_Y;

    deltaXLogger.append(deltaX);
    deltaYLogger.append(deltaY);

    estimatedPose = Optional.of(new Pose2d(robotPose.getX() + deltaX, robotPose.getY() + deltaY, new Rotation2d()));

    return estimatedPose;
  }

  @Override
  public void updatePoseEstimate(SwerveDrivePoseEstimator estimator, Pose3d targetPose) {
    if (hasTargets()) {
      Optional<Pose2d> estimatedPose = getEstimatedRobotPose(estimator.getEstimatedPosition(), targetPose);

      if (estimatedPose.isPresent()) {
        estimator.addVisionMeasurement(estimatedPose.get(), getTargetTimestamp());
      }
    }

    targetXLogger.append(targetPose.getX());
    targetYLogger.append(targetPose.getY());
    targetAngleLogger.append(Math.toRadians(targetPose.getRotation().getAngle()));
  }

  @Override
  public void periodic() {
    super.periodic();

    if (estimateForShuffleboard.get()) {
      getEstimatedRobotPose(TEST_ROBOT_POSE, TEST_TARGET_POSE);
    }
  }
}
