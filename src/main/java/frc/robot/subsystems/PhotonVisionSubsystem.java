// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.cscore.HttpCamera;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for getting target information from
 * PhotonVision.
 */
@RobotPreferencesLayout(groupName = "PhotonVision", row=0, column = 6, width = 2, height = 1)
public class PhotonVisionSubsystem extends SubsystemBase {
  @RobotPreferencesValue
  public static final RobotPreferences.BooleanValue enableTab = new RobotPreferences.BooleanValue("PhotonVision", "Enable Tab", false);

  private final PhotonCamera camera = new PhotonCamera("IMX219");
  private PhotonPipelineResult result = new PhotonPipelineResult();

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();

  }

  /**
   * Returns whether the result contains any targets.
   * 
   * @return Returns true if there are targets.
   */
  public boolean hasTargets() {
    return result.hasTargets();
  }

  /**
   * Returns information on the best target.
   * 
   * @return Information on the best target.
   */
  public PhotonTrackedTarget getBestTarget() {
    return result.getBestTarget();
  }

  public double getDistanceToBestTarget(){
    if(!hasTargets()){
      return 0;
    }

    Transform3d bestTarget = getBestTarget().getBestCameraToTarget();
    return Math.hypot(bestTarget.getX(), bestTarget.getY());
  }

  public double getBestAngleToTarget(){
    if(!hasTargets()){
      return 0;
    }

    return getBestTarget().getYaw();
  }

  /**
   * Returns a list of visible targets.
   * 
   * @return A list of visible targets.
   */
  public List<PhotonTrackedTarget> getTargets() {
    return result.getTargets();
  }

  public void addShuffleboardTab(){
    if(!enableTab.getValue()){
      return;
    }
    ShuffleboardTab visionTab = Shuffleboard.getTab("PhotonVision");
    ShuffleboardLayout targetLayout = visionTab.getLayout("Target Info", BuiltInLayouts.kList)
        .withPosition(0, 0)
        .withSize(2, 3);
    targetLayout.addBoolean("Has Target", this::hasTargets);
    targetLayout.addDouble("Distance", this::getDistanceToBestTarget);
    targetLayout.addDouble("Angle", this::getBestAngleToTarget);

    VideoSource video = new HttpCamera("PhotonVision", "http://photonvision.local:1181/stream.mjpg");
    visionTab.add("PhotonVision", video)
        .withWidget(BuiltInWidgets.kCameraStream)
        .withPosition(2, 0)
        .withSize(4, 3);

  }



}
