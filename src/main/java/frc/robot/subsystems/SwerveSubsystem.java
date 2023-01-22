// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.drive.SwerveDrive;
import frc.robot.drive.SwerveModule;
import frc.robot.parameters.SwerveDriveParameters;

public class SwerveSubsystem extends SubsystemBase {
  public static SwerveDriveParameters PARAMETERS = SwerveDriveParameters.Competition2022;

  private static final byte kNavXUpdateFrequencyHz = 50;

  // 4 pairs of motors for drive & steering.
  private final WPI_TalonFX frontLeftDriveMotor = new WPI_TalonFX(1);
  private final WPI_TalonFX frontLeftSteeringMotor = new WPI_TalonFX(2);

  private final WPI_TalonFX frontRightDriveMotor = new WPI_TalonFX(3);
  private final WPI_TalonFX frontRightSteeringMotor = new WPI_TalonFX(4);

  private final WPI_TalonFX backLeftDriveMotor = new WPI_TalonFX(7);
  private final WPI_TalonFX backLeftSteeringMotor = new WPI_TalonFX(8);

  private final WPI_TalonFX backRightDriveMotor = new WPI_TalonFX(5);
  private final WPI_TalonFX backRightSteeringMotor = new WPI_TalonFX(6);

  // 4 CANcoders for the steering angle.
  private final CANCoder frontLeftAngle = new CANCoder(9);
  private final CANCoder frontRightAngle = new CANCoder(10);
  private final CANCoder backLeftAngle = new CANCoder(12);
  private final CANCoder backRightAngle = new CANCoder(11);

  private final SwerveModule frontLeftModule = createSwerveModule(
      frontLeftDriveMotor, frontLeftSteeringMotor, frontLeftAngle, "Front Left");
  private final SwerveModule frontRightModule = createSwerveModule(
      frontRightDriveMotor, frontRightSteeringMotor, frontRightAngle, "Front Right");
  private final SwerveModule backLeftModule = createSwerveModule(
      backLeftDriveMotor, backLeftSteeringMotor, backLeftAngle, "Back Left");
  private final SwerveModule backRightModule = createSwerveModule(
      backRightDriveMotor, backRightSteeringMotor, backRightAngle, "Back Right");

  private final SwerveModule[] modules = { frontLeftModule, frontRightModule, backLeftModule, backRightModule };

  private final AHRS ahrs = new AHRS(SPI.Port.kMXP, kNavXUpdateFrequencyHz);

  SwerveDriveKinematics kinematics = PARAMETERS.getKinematics();

  private final SwerveDrive drivetrain = new SwerveDrive(
      PARAMETERS, modules, () -> -ahrs.getAngle());

  private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      kinematics, getRotation2d(), drivetrain.getModulesPositions());

  /**
   * Creates a {@link SwerveModule} object and intiailizes its motor controllers.
   * 
   * @param driveMotor    The drive motor controller.
   * @param steeringMotor The steering motor controller.
   * @param wheelAngle    An absolute encoder that measures the wheel angle.
   * @param name          The name of the module.
   * 
   * @return An initialized {@link SwerveModule} object.
   */
  private static SwerveModule createSwerveModule(WPI_TalonFX driveMotor, WPI_TalonFX steeringMotor, CANCoder wheelAngle,
      String name) {
    driveMotor.setNeutralMode(NeutralMode.Brake);
    steeringMotor.setNeutralMode(NeutralMode.Brake);
    wheelAngle.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);

    final double drivePulsesPerMeter = PARAMETERS.getDrivePulsesPerMeter();

    return new SwerveModule(
        PARAMETERS,
        driveMotor,
        () -> {
          return driveMotor.getSelectedSensorPosition() / drivePulsesPerMeter;
        },
        // The WPI_TalonFX reports the velocity in pulses per 100ms, so we need to
        // multiply by 10 to convert to pulses per second.
        () -> (driveMotor.getSelectedSensorVelocity() * 10) / drivePulsesPerMeter,
        steeringMotor,
        () -> Rotation2d.fromDegrees(wheelAngle.getAbsolutePosition()),
        name);
  }

  /** Creates a new SwerveSubsystem. */
  public SwerveSubsystem() {
    ahrs.reset();
    drivetrain.setDeadband(0.1);
  }

  /**
   * Returns the maximum drive speed in m/s of a swerve module.
   * 
   * @return The maximum drive speed.
   */
  public double getMaxSpeed() {
    return PARAMETERS.getMaxDriveSpeed();
  }

  /**
   * Returns the maximum drive acceleration in m/s^2 of a swerve module.
   * 
   * @return The maximum drive acceleration.
   */
  public double getMaxAcceleration() {
    return PARAMETERS.getMaxDriveAcceleration();
  }

  /**
   * Returns the swerve drive kinematics for this subsystem.
   * 
   * @return The swerve drive kinematics.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns a {@link SwerveDriveKinematicsConstraint} object used to enforce
   * swerve drive kinematics constraints when following a trajectory.
   * 
   * @return A {@link SwerveDriveKinematicsConstraint} object used to enforce
   *         swerve drive kinematics constraints when following a trajectory.
   */
  public SwerveDriveKinematicsConstraint getKinematicsConstraint() {
    return PARAMETERS.getKinematicsConstraint();
  }

  /**
   * Returns a {@link TrapezoidProfile.Constraints} object used to enforce
   * velocity and acceleration constraints on the {@link ProfiledPIDController}
   * used to reach the goal robot orientation.
   * 
   * @return A {@link TrapezoidProfile.Constraints} object used to enforce
   *         velocity and acceleration constraints on the controller used to reach
   *         the goal robot orientation.
   */
  public TrapezoidProfile.Constraints getRotationalConstraints() {
    return PARAMETERS.getRotationalConstraints();
  }

  /**
   * Creates a HolonomicDriveController for the subsystem.
   * 
   * @return A HolonomicDriveController.
   */
  public HolonomicDriveController createDriveController() {
    ProfiledPIDController thetaController = new ProfiledPIDController(
        1.0, 0.0, 0.0, getRotationalConstraints());

    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return new HolonomicDriveController(
        new PIDController(1.0, 0.0, 0.0),
        new PIDController(1.0, 0.0, 0.0),
        thetaController);
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

  /**
   * Sets the swerve module states.
   * 
   * @param states An array of four {@link SwerveModuleState} objects in the
   *               order: front left, front right, back left, back right
   */
  public void setModuleStates(SwerveModuleState[] states) {
    drivetrain.setModuleStates(states);
  }

  // Stops motors from the subsystem - may need to remove this (not sure - Om)
  public void stopMotors() {
    drivetrain.stopMotor();
  }

  /**
   * Resets the robots position on the field.
   * 
   * @param initialPosition Sets the initial position.
   */
  public void resetPosition(Pose2d initialPosition) {
    odometry.resetPosition(getRotation2d(), drivetrain.getModulesPositions(), initialPosition);
  }

  /**
   * Return current position & orientation of the robot on the field.
   * 
   * @return The current position and orientation of the robot.
   */
  public Pose2d getPosition() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the field orientation of the robot as a {@link Rotation2d} object.
   * 
   * @return Gets the field orientation of the robot.
   */
  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(-ahrs.getAngle());
  }

  /**
   * Returns the tilt of the robot as a {@link Rotation2d} object.
   * 
   * @return Gets the tilt of the robot (positive is nose up, negative is nose
   *         down).
   */
  public Rotation2d getTilt() {
    return Rotation2d.fromDegrees(-ahrs.getRoll());
  }

  @Override
  public void periodic() {
    odometry.update(getRotation2d(), drivetrain.getModulesPositions());
  }

  /**
   * Adds a tab for swerve drive in Shuffleboard.
   */
  public void addShuffleboardTab() {
    ShuffleboardTab swerveDriveTab = Shuffleboard.getTab("Drive");
    drivetrain.addShuffleboardLayouts(swerveDriveTab);

    ShuffleboardLayout layout = swerveDriveTab.getLayout("Odometry", BuiltInLayouts.kList)
        .withPosition(6, 0)
        .withSize(3, 4);
    layout.add("Gyro", new Sendable() {

      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty("Value", () -> odometry.getPoseMeters().getRotation().getDegrees(), null);
      }
    }).withWidget(BuiltInWidgets.kGyro);
    layout.addDouble("x", () -> odometry.getPoseMeters().getX());
    layout.addDouble("y", () -> odometry.getPoseMeters().getY());

    layout.addDouble("pitch", () -> ahrs.getPitch());
    layout.addDouble("roll", () -> ahrs.getRoll());

  }
}
