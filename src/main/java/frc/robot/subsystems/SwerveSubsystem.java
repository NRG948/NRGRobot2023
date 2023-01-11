// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.motors.TalonFXMotorController;

public class SwerveSubsystem extends SubsystemBase {

  // Constants for motor locations
  private static final double TRACK_LENGTH = 0.6604;
  private static final double TRACK_WIDTH = 0.4826;

  private static final double WHEEL_RADIUS = 0.047625; // Meters
  private static final int ENCODER_RESOLUTION = 2048; // Steps per Rev
  private static final double DRIVE_GEAR_RATIO = 8.14; // Gear ratio
  private static final double DRIVE_PULSES_PER_METER = (ENCODER_RESOLUTION * DRIVE_GEAR_RATIO)
      / (2 * WHEEL_RADIUS * Math.PI); // pulses per meter

  private static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(TRACK_LENGTH / 2, TRACK_WIDTH / 2);
  private static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(TRACK_LENGTH / 2, -TRACK_WIDTH / 2);
  private static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-TRACK_LENGTH / 2, TRACK_WIDTH / 2);
  private static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-TRACK_LENGTH / 2, -TRACK_WIDTH / 2);

  private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      FRONT_LEFT_LOCATION,
      FRONT_RIGHT_LOCATION,
      BACK_LEFT_LOCATION,
      BACK_RIGHT_LOCATION);

  // 4 pairs of motors for drive & steering.
  private final TalonFX frontLeftDriveMotor = new TalonFX(1);
  private final TalonFX frontLeftSteeringMotor = new TalonFX(2);

  private final TalonFX frontRightDriveMotor = new TalonFX(3);
  private final TalonFX frontRightSteeringMotor = new TalonFX(4);

  private final TalonFX backLeftDriveMotor = new TalonFX(7);
  private final TalonFX backLeftSteeringMotor = new TalonFX(8);

  private final TalonFX backRightDriveMotor = new TalonFX(5);
  private final TalonFX backRightSteeringMotor = new TalonFX(6);

  // 4 CANcoders for the steering angle.
  private final CANCoder frontLeftAngle = new CANCoder(9);
  private final CANCoder frontRightAngle = new CANCoder(10);
  private final CANCoder backLeftAngle = new CANCoder(12);
  private final CANCoder backRightAngle = new CANCoder(11);

  private final SwerveModule frontLeftModule = CreateSwerveModule(
      frontLeftDriveMotor, frontLeftSteeringMotor, frontLeftAngle);
  private final SwerveModule frontRightModule = CreateSwerveModule(
      frontRightDriveMotor, frontRightSteeringMotor, frontRightAngle);
  private final SwerveModule backLeftModule = CreateSwerveModule(
      backLeftDriveMotor, backLeftSteeringMotor, backLeftAngle);
  private final SwerveModule backRightModule = CreateSwerveModule(
      backRightDriveMotor, backRightSteeringMotor, backRightAngle);

  private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

  private final AHRS ahrs = new AHRS(SerialPort.Port.kMXP);

  private final SwerveDrive drivetrain = new SwerveDrive(modules, kinematics, () -> ahrs.getAngle());

  /**
   * Creates a {@link SwerveModule} object and intiailizes its motor controllers.
   * 
   * @param driveMotor    The drive motor controller.
   * @param steeringMotor The steering motor controller.
   * @param wheelAngle    An absolute encoder that measures the wheel angle.
   * 
   * @return An initialized {@link SwerveModule} object.
   */
  private static SwerveModule CreateSwerveModule(TalonFX driveMotor, TalonFX steeringMotor, CANCoder wheelAngle) {
    driveMotor.setNeutralMode(NeutralMode.Brake);
    steeringMotor.setNeutralMode(NeutralMode.Brake);

    TalonFXMotorController driveController = new TalonFXMotorController(driveMotor);
    TalonFXMotorController steeringController = new TalonFXMotorController(steeringMotor);

    return new SwerveModule(driveController,
        // The TalonFX reports the velocity in pulses per 100ms, so we need to
        // multiply by 10 to convert to pulses per second.
        () -> (driveMotor.getSelectedSensorVelocity() * 10) / DRIVE_PULSES_PER_METER, steeringController,
        () -> wheelAngle.getAbsolutePosition());
  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {

  }

  /**
   * Drives the robot based on joystick inputs.
   * 
   * @param xSpeed        Speed of the robot in the x direction.
   * @param ySpeed        Speed of the robot in the y direction.
   * @param rSpeed        Rotation speed of the robot.
   * @param fieldRelative Whether the x and y values are relative to field.
   * @param squareInputs  Decreases sensitivity at low speeds.
   */
  public void drive(double xSpeed, double ySpeed, double rSpeed, boolean fieldRelative, boolean squareInputs) {
    drivetrain.drive(xSpeed, ySpeed, rSpeed, fieldRelative, squareInputs);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
