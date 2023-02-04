// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem is responsible for getting target information from
 * PhotonVision.
 */
public class PhotonVisionSubsystem extends SubsystemBase {
  private final PhotonCamera camera = new PhotonCamera("photonvision");
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

  /**
   * Returns a list of visible targets.
   * 
   * @return A list of visible targets.
   */
  public List<PhotonTrackedTarget> getTargets() {
    return result.getTargets();
  }
}
