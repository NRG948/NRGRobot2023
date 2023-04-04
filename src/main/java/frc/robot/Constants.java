// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  /**
   * Defines constant values for robot components.
   */
  public static class RobotConstants {
    /**
     * The maximum battery voltage.
     */
    public static final double MAX_BATTERY_VOLTAGE = 12.0;

    /**
     * PWM Ports.
     */
    public static class PWMPort {

      
      public static final int LED = 1;
      public static final int LightningLED = 2;

    }

    /**
     * CAN Ids.
     */
    public static class CAN {
      public static class SparkMax {
        public static final int INTAKE = 17;
        public static final int INDEXER = 4;
        public static final int TOP_SHOOTER = 3;
        public static final int BOTTOM_SHOOTER = 2;
      }
    }

    /**
     * Digital I/O port numbers.
     */
    public static class DigitalIO {
      public static final int INDEXER_BEAM_BREAK = 0; //check with systems if beam break is applicable
    }

    /**
     * A transform from the front camera to the center of the robot.
     */
    public static final Transform3d FRONT_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(Units.inchesToMeters(-6.5), Units.inchesToMeters(0), Units.inchesToMeters(-26)),
        new Rotation3d(0, Math.toRadians(-60), 0));

    /**
     * A transform from the robot center to the front camera.
     */
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = FRONT_CAMERA_TO_ROBOT.inverse();

    /**
     * A transform from the back camera to the center of the robot.
     */
    public static final Transform3d BACK_CAMERA_TO_ROBOT = new Transform3d(
        new Translation3d(Units.inchesToMeters(-8), Units.inchesToMeters(6.5), Units.inchesToMeters(-15.125)),
        new Rotation3d(0, Math.toRadians(0), Math.toRadians(180)));

    /**
     * A transform from the robot center to the back camera.
     */
    public static final Transform3d ROBOT_TO_BACK_CAMERA = BACK_CAMERA_TO_ROBOT.inverse();

    /**
     * A constant representing the distance from the grid to position the robot for
     * scoring a game element.
     */
    public static final double SCORING_DISTANCE_FROM_GRID = Units.inchesToMeters(16);

    /**
     * The robot length including bumpers.
     * 
     * TODO: Determine competition robot length.
     */
    public static final double ROBOT_LENGTH = Units.inchesToMeters(39);

    /**
     * The offset from the center of the grid to side scoring position.
     */
    public static final double GRID_SIDE_OFFSET = Units.inchesToMeters(22);
  }

  /**
   * Defines operator (i.e. driver and manipulator) constants.
   */
  public static class OperatorConstants {

    /**
     * Defines the port numbers of the Xbox controllers.
     */
    public static class XboxControllerPort {

      public static final int DRIVER = 0;
      public static final int MANIPULATOR = 1;

    }
  }

  public static class ColorConstants {
    public static final Color8Bit BLACK = new Color8Bit(0, 0, 0);
    public static final Color8Bit RED = new Color8Bit(204, 0, 0);
    public static final Color8Bit ORANGE = new Color8Bit(204, 84, 0);
    public static final Color8Bit YELLOW = new Color8Bit(204, 204, 0);
    public static final Color8Bit GREEN = new Color8Bit(0, 204, 0);
    public static final Color8Bit BLUE = new Color8Bit(0, 0, 204);
    public static final Color8Bit PURPLE = new Color8Bit(152, 16, 201);
    public static final Color8Bit COLORS[] = { RED, ORANGE, YELLOW, GREEN, BLUE, PURPLE };
  }
}
