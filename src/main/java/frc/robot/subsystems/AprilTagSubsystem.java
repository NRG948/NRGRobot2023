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

@RobotPreferencesLayout(groupName = "AprilTag", row = 1, column = 4, width = 2, height = 1)
public class AprilTagSubsystem extends PhotonVisionSubsystemBase {

  private static final Transform3d TAG_TO_ROBOT = new Transform3d(
      new Translation3d(Units.inchesToMeters(33), 0, 0),
      new Rotation3d(0, 0, 0));

  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue(
      "AprilTag", "Enable Tab", false);

  private DoubleLogEntry targetXLogger = new DoubleLogEntry(DataLogManager.getLog(), "AprilTag/Target X");
  private DoubleLogEntry targetYLogger = new DoubleLogEntry(DataLogManager.getLog(), "AprilTag/Target Y");
  private DoubleLogEntry targetAngleLogger = new DoubleLogEntry(DataLogManager.getLog(), "AprilTag/Target Angle");

  /** Creates a new PhotonVisionSubsystem. */
  public AprilTagSubsystem() {
    super("Back", RobotConstants.BACK_CAMERA_TO_ROBOT, TAG_TO_ROBOT);
  }

  /**
   * Adds a tab for April Tag in Shuffleboard.
   */
  public void addShuffleboardTab() {
    if (!enableTab.getValue()) {
      return;
    }

    ShuffleboardTab visionTab = Shuffleboard.getTab("April Tag");
    ShuffleboardLayout targetLayout = visionTab.getLayout("Target Info", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 3);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Distance", this::getDistanceToBestTarget);
    targetLayout.addDouble("Angle", this::getAngleToBestTarget);

    VideoSource video = new HttpCamera("photonvision_Port_1182_MJPEG_Server", "http://10.9.48.11:1182/?action=stream",
        HttpCameraKind.kMJPGStreamer);
    visionTab.add("April Tag", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);
  }

  @Override
  public void updatePoseEstimate(SwerveDrivePoseEstimator estimator, Pose3d targetPose) {
    if (hasTargets()) {
      Transform3d cameraToRobot = getCameraToRobotTransform();
      Transform3d targetToCamera = new Transform3d(
          new Translation3d(
              getDistanceToBestTarget(),
              new Rotation3d(0, 0, Math.toRadians(-getAngleToBestTarget()))),
          cameraToRobot.getRotation()).inverse();
      Pose3d cameraPose = targetPose.transformBy(targetToCamera);
      Pose3d robotPose = cameraPose.transformBy(cameraToRobot);

      estimator.addVisionMeasurement(robotPose.toPose2d(), getTargetTimestamp());
    }

    targetXLogger.append(targetPose.getX());
    targetYLogger.append(targetPose.getY());
    targetAngleLogger.append(Math.toRadians(targetPose.getRotation().getAngle()));
  }
}
