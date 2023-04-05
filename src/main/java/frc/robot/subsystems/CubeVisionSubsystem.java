// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants.RobotConstants;

/**
 * This subsystem is responsible for getting target information from
 * PhotonVision.
 */
@RobotPreferencesLayout(groupName = "CubeVision", row = 0, column = 4, width = 2, height = 1)
public class CubeVisionSubsystem extends PhotonVisionSubsystemBase {
  /**
   *
   */
  private static final Transform3d TARGET_TO_ROBOT = new Transform3d(
    new Translation3d(Units.inchesToMeters(-12.0), 0,0),
    new Rotation3d()
  );

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue(
      "CubeVision", "Enable Tab", false);

  private DoubleLogEntry deltaXLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Delta X");
  private DoubleLogEntry deltaYLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Delta Y");
  private DoubleLogEntry targetXLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Target X");
  private DoubleLogEntry targetYLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Target Y");
  private DoubleLogEntry targetAngleLogger = new DoubleLogEntry(DataLogManager.getLog(), "Cube/Target Angle");

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
    ShuffleboardLayout targetLayout = visionTab.getLayout("Target Info", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 3);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Distance", this::getDistanceToBestTarget);
    targetLayout.addDouble("Angle", this::getAngleToBestTarget);

    VideoSource video = new HttpCamera("photonvision_Port_1183_MJPEG_Server", "http://10.9.48.11:1183/?action=stream",
        HttpCameraKind.kMJPGStreamer);
    visionTab.add("CubeVision", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);
  }

  @Override
  public void updatePoseEstimate(SwerveDrivePoseEstimator estimator, Pose3d targetPose) {
    if (hasTargets()) {
      Pose3d currentCameraPose = new Pose3d(estimator.getEstimatedPosition()).transformBy(getRobotToCameraTransform());
      double deltaX = targetPose.getX() - currentCameraPose.getX();

      if (deltaX > 0) {
        double deltaY = deltaX * Math.tan(Math.toRadians(-getAngleToBestTarget()));
        Transform3d targetToCamera = new Transform3d(
            new Translation3d(deltaX, deltaY, 0),
            getCameraToRobotTransform().getRotation()).inverse();
        Pose3d cameraPose = targetPose.transformBy(targetToCamera);
        Pose2d robotPose = cameraPose.transformBy(getCameraToRobotTransform()).toPose2d();

        estimator.addVisionMeasurement(robotPose, getTargetTimestamp());

        deltaXLogger.append(deltaX);
        deltaYLogger.append(deltaY);
      }
    }
    
    targetXLogger.append(targetPose.getX());
    targetYLogger.append(targetPose.getY());
    targetAngleLogger.append(Math.toRadians(targetPose.getRotation().getAngle()));
  }
}
