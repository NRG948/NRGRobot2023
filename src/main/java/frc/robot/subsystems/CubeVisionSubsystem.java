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
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue(
      "CubeVision", "Enable Tab", false);
      
  /** Creates a new PhotonVisionSubsystem. */
  public CubeVisionSubsystem() {
    super("Front",RobotConstants.FRONT_CAMERA_TO_ROBOT);
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
}
