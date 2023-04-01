// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.nrg948.preferences.RobotPreferencesLayout;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for getting target information from
 * PhotonVision.
 */
@RobotPreferencesLayout(groupName = "PhotonVision", row = 0, column = 4, width = 2, height = 1)
public class PhotonVisionSubsystemBase extends SubsystemBase {

  private final PhotonCamera camera;
  private final Transform3d cameraToRobot;
  private final Transform3d robotToCamera;
  private final Transform3d targetToRobot;
  private PhotonPipelineResult result = new PhotonPipelineResult();

  private BooleanLogEntry hasTargetLogger;
  private DoubleLogEntry distanceLogger;
  private DoubleLogEntry angleLogger;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystemBase(String cameraName, Transform3d cameraToRobot, Transform3d targetToRobot) {
    camera = new PhotonCamera(cameraName);
    this.cameraToRobot = cameraToRobot;
    this.robotToCamera = cameraToRobot.inverse();
    this.targetToRobot = targetToRobot;
    hasTargetLogger = new BooleanLogEntry(DataLogManager.getLog(), cameraName + "/Has Target");
    distanceLogger = new DoubleLogEntry(DataLogManager.getLog(), cameraName + "/Distance");
    angleLogger = new DoubleLogEntry(DataLogManager.getLog(), cameraName + "/Angle");
  }

  @Override
  public void periodic() {
    result = camera.getLatestResult();
    hasTargetLogger.append(hasTargets());
    distanceLogger.append(getDistanceToBestTarget());
    angleLogger.append(-getAngleToBestTarget());
  }

  public Transform3d getCameraToRobotTransform() {
    return cameraToRobot;
  }

  public Transform3d getRobotToCameraTransform() {
    return robotToCamera;
  }

  public Transform3d getTargetToRobotTransform() {
    return targetToRobot;
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

  public double getTargetTimestamp() {
    return result.getTimestampSeconds();
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
