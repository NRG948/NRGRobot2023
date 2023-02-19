// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.io.File;
import java.io.FilenameFilter;

/** A class containing utility methods for the {@link File} class. */
public class FileUtil {
  /**
   * Returns a {@link FilenameFilter} that includes filenames with the specified
   * extension.
   * 
   * @param extension The extension to include; it must include the leading '.'
   *                  character.
   * 
   * @return A filter the includes filenames with the specified extension.
   */
  public static FilenameFilter withExtension(String extension) {
    return (file, name) -> name.endsWith(extension);
  }

  /**
   * Returns the basename of the file.
   * 
   * @param file The file.
   * 
   * @return The basename of the file.
   */
  public static String basenameOf(File file) {
    String fileName = file.getName();

    return fileName.substring(0, fileName.lastIndexOf("."));
  }
}
