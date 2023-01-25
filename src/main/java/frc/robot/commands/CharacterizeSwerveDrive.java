// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.sysid.SysIdGeneralMechanismLogger;

@AutonomousCommand(name = "Characterize Swerve Drive")
public class CharacterizeSwerveDrive extends CommandBase {
  private final SwerveSubsystem drivetrain;
  private final SysIdGeneralMechanismLogger logger;

  /** Creates a new CharacterizeSwerveDrive. */
  public CharacterizeSwerveDrive(Subsystems subsystems) {
    this.drivetrain = subsystems.drivetrain;
    this.logger = new SysIdGeneralMechanismLogger(
        () -> this.drivetrain.getPosition().getX(),
        () -> this.drivetrain.getChassisSpeeds().vxMetersPerSecond);
    addRequirements(this.drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.resetPosition(new Pose2d());
    logger.init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    logger.logData();
    double voltage = logger.getMotorVoltage();
    ChassisSpeeds speeds = new ChassisSpeeds(
        (voltage / RobotConstants.kMaxBatteryVoltage) * drivetrain.getMaxSpeed(),
        0.0,
        0.0);
    drivetrain.setChassisSpeeds(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stopMotors();
    logger.sendData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
