// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.preferences.RobotPreferences;
import com.nrg948.preferences.RobotPreferencesLayout;
import com.nrg948.preferences.RobotPreferencesValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * A command that balances the robot on the charging station. It assumes the
 * robot has been driven at least partially onto the charging station.
 */
@RobotPreferencesLayout(groupName = "Auto Balance", row = 0, column = 2, width = 2, height = 3)
public class AutoBalanceOnChargeStation extends CommandBase {
  private double SIN_45 = Math.sin(Math.toRadians(45));
  private static final String PREFERENCES_GROUP = "Auto Balance";

  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INITIAL_SPEED_PERCENT = new RobotPreferences.DoubleValue(
      PREFERENCES_GROUP, "Initial Speed", 0.35);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue MAX_SPEED_PERCENT = new RobotPreferences.DoubleValue(
      PREFERENCES_GROUP, "Speed", 0.20);

  /*
   * The PID constants were derived using the Zeigler-Nichols tuning method. We
   * started with a kP of 0.07 and measured an oscillation period of 2.26733
   * seconds.
   */
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue ANGLE_KP = new RobotPreferences.DoubleValue(
      PREFERENCES_GROUP, "KP", 0.021);
  //@RobotPreferencesValue
  //public static final RobotPreferences.DoubleValue ANGLE_KI = new RobotPreferences.DoubleValue(
  //    PREFERENCES_GROUP, "KI", 0.037047927);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue ANGLE_KD = new RobotPreferences.DoubleValue(
      PREFERENCES_GROUP, "KD", 0.0060);

  private static final double TIME_AT_LEVEL = 0.33;

  private final SwerveSubsystem drivetrain;
  private final Timer timer = new Timer();

  private PIDController anglePID;
  private boolean wasLevel; // Was the robot "level" during the last command execution
  private double maxSpeed;

  /** 
   * Creates a new AutoBalanceOnChargeStation command.
   * 
   * Robot must already be on the Charge Station before this command is invoked.
   */
  public AutoBalanceOnChargeStation(SwerveSubsystem drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    maxSpeed = this.drivetrain.getMaxSpeed() * INITIAL_SPEED_PERCENT.getValue();

    anglePID = new PIDController(ANGLE_KP.getValue(), 0, ANGLE_KD.getValue());
    anglePID.setSetpoint(0);
    anglePID.setTolerance(2.0);
    anglePID.reset();
    timer.reset();

    wasLevel = false;
    System.out.println("Starting AutoBalanceOnChargeStation at tilt: " + drivetrain.getTilt().getDegrees());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measuredAngle = drivetrain.getTilt().getDegrees();

    if (Math.abs(measuredAngle) <= 2.0) {
      maxSpeed = this.drivetrain.getMaxSpeed() * MAX_SPEED_PERCENT.getValue();
    }

    double speed = -anglePID.calculate(measuredAngle) * maxSpeed;
    SmartDashboard.putNumber("Tilt", measuredAngle);
    SmartDashboard.putNumber("Speed", speed);

    if (anglePID.atSetpoint()) {
      drivetrain.stopMotors();
      return;
    }
    
    double xSpeed, ySpeed;
    if (Math.abs(measuredAngle) < 10) {
      xSpeed = speed * SIN_45;
      ySpeed = xSpeed;
    } else {
      xSpeed = speed;
      ySpeed = 0;
    }
    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        xSpeed, ySpeed, 0.0, drivetrain.getOrientation());

    drivetrain.setChassisSpeeds(chassisSpeeds, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    timer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean level = anglePID.atSetpoint();

    if (level != wasLevel) {
      if (!level) {
        timer.stop();
      } else {
        timer.reset();
        timer.start();
      }

      wasLevel = level;
    }

    return timer.hasElapsed(TIME_AT_LEVEL);
  }
}
