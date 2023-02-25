// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommandMethod;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Subsystems;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.sysid.SysIdGeneralMechanismLogger;
import frc.robot.util.SwerveModuleVoltages;

/**
 * An autonomous command to integrate with SysId to characterize the swerve
 * steering.
 * <p>
 * Use the SysId tool to characterize the swerve drive as a "Simple" mechanism.
 */
public class CharacterizeSwerveSteering extends CommandBase {
  private final SwerveSubsystem drivetrain;
  private final SysIdGeneralMechanismLogger logger;

  /**
   * Return a command to characterize swerve steering module 0.
   * 
   * @param subsystems The subsystems container.
   * @return a command to characterize swerve steering module 0.
   */
  @AutonomousCommandMethod(name = "[SYSID] Characterize Swerve Steering Module 0")
  public static Command forModule0(Subsystems subsystems) {
    return new CharacterizeSwerveSteering(subsystems, 0);
  }

  /**
   * Return a command to characterize swerve steering module 1.
   * 
   * @param subsystems The subsystems container.
   * @return a command to characterize swerve steering module 1.
   */
  @AutonomousCommandMethod(name = "[SYSID] Characterize Swerve Steering Module 1")
  public static Command forModule1(Subsystems subsystems) {
    return new CharacterizeSwerveSteering(subsystems, 1);
  }

  /**
   * Return a command to characterize swerve steering module 2.
   * 
   * @param subsystems The subsystems container.
   * @return a command to characterize swerve steering module 2.
   */
  @AutonomousCommandMethod(name = "[SYSID] Characterize Swerve Steering Module 2")
  public static Command forModule2(Subsystems subsystems) {
    return new CharacterizeSwerveSteering(subsystems, 2);
  }

  /**
   * Return a command to characterize swerve steering module 3.
   * 
   * @param subsystems The subsystems container.
   * @return a command to characterize swerve steering module 3.
   */
  @AutonomousCommandMethod(name = "[SYSID] Characterize Swerve Steering Module 3")
  public static Command forModule3(Subsystems subsystems) {
    return new CharacterizeSwerveSteering(subsystems, 3);
  }

  /**
   * Creates a new CharacterizeSwerveSteering.
   * 
   * @param subsystems   The subsystems container.
   * @param moduleNumber The module to characterize.
   */
  public CharacterizeSwerveSteering(Subsystems subsystems, int moduleNumber) {
    this.drivetrain = subsystems.drivetrain;
    this.logger = new SysIdGeneralMechanismLogger(
        () -> drivetrain.getModulePositions()[moduleNumber].angle.getRadians(),
        () -> drivetrain.getModuleVelocities()[moduleNumber].steeringVelocity);

    addRequirements(subsystems.drivetrain);
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

    SwerveModuleVoltages[] voltages = new SwerveModuleVoltages[] {
        new SwerveModuleVoltages(0.0, voltage),
        new SwerveModuleVoltages(0.0, voltage),
        new SwerveModuleVoltages(0.0, voltage),
        new SwerveModuleVoltages(0.0, voltage) };
    drivetrain.setModuleVoltages(voltages);
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
