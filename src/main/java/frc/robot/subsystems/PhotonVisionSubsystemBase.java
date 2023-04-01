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
import edu.wpi.first.cscore.HttpCamera.HttpCameraKind;
import edu.wpi.first.math.geometry.Transform3d;
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
@RobotPreferencesLayout(groupName = "PhotonVision", row = 0, column = 4, width = 2, height = 1)
public class PhotonVisionSubsystemBase extends SubsystemBase {

  private final PhotonCamera camera;
  private PhotonPipelineResult result = new PhotonPipelineResult();

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystemBase(String cameraName) {
    camera = new PhotonCamera(cameraName);
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

  /**
   * Returns the distance to the best target.
   * 
   * @return The distance, in meters, to the best target.
   */
  public double getDistanceToBestTarget() {
    if (!hasTargets()) {
      return 0;
    }

    Transform3d bestTarget = getBestTarget().getBestCameraToTarget();
    return Math.hypot(bestTarget.getX(), bestTarget.getY());
  }

  /**
   * Returns the angle to the best target.
   * 
   * @return The angle to the best target.
   */
  public double getAngleToBestTarget() {
    if (!hasTargets()) {
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
}
