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
public class AutoBalanceOnChargeStation2 extends CommandBase {
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue INITIAL_SPEED_PERCENT = new RobotPreferences.DoubleValue(
      "Auto Balance", "Initial Speed", 0.35);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue MAX_SPEED_PERCENT = new RobotPreferences.DoubleValue(
      "Auto Balance", "Speed", 0.15);

  /*
   * The PID constant were derived using the Zeigler-Nichols tuning method. We
   * started with a kP of 0.7 and measured an oscillation period of 2.26733
   * seconds.
   */
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue ANGLE_KP = new RobotPreferences.DoubleValue(
      "Auto Balance", "KP", 0.042);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue ANGLE_KI = new RobotPreferences.DoubleValue(
      "Auto Balance", "KI", 0.037047927);
  @RobotPreferencesValue
  public static final RobotPreferences.DoubleValue ANGLE_KD = new RobotPreferences.DoubleValue(
      "Auto Balance", "KD", 0.0119035);

  private static final double TIME_AT_LEVEL = 0.2;

  private final SwerveSubsystem drivetrain;
  private final Timer timer = new Timer();

  private PIDController anglePID;
  private boolean wasLevel = false;
  private double maxSpeed;

  /** Creates a new AutoBalanceOnChargeStation2. */
  public AutoBalanceOnChargeStation2(SwerveSubsystem drivetrain) {
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double measuredAngle = drivetrain.getTilt().getDegrees();
    if (measuredAngle <= 2.0) {
      maxSpeed = MAX_SPEED_PERCENT.getValue();
    }
    double speed = -anglePID.calculate(measuredAngle) * maxSpeed;
    SmartDashboard.putNumber("Tilt", measuredAngle);
    SmartDashboard.putNumber("Speed", speed);
    if (anglePID.atSetpoint()) {
      drivetrain.stopMotors();
      return;
    }

    ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        speed, 0.0, 0.0, drivetrain.getOrientation());

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
