// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;

/** A class containing utility methods for the {@link Filesystem} class. */
public class FilesystemUtil {

  /**
   * Obtains the 'pathplanner' directory, located in the 'deploy' directory. See
   * {@link Filesystem#getDeployDirectory()} for the location of the 'deploy'
   * directory.
   * 
   * @return The 'pathplanner' directory.
   */
  public static File getPathplannerDirectory() {
    return new File(Filesystem.getDeployDirectory(), "pathplanner");
  }
}
