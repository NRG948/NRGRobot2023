// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorAngleSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.sysid.SysIdGeneralMechanismLogger;

@AutonomousCommand(name = "[SYSID] Characterize Elevator Angle")
public class CharacterizeElevatorAngle extends CommandBase {
  private ElevatorAngleSubsystem elevatorAngle;
  private SysIdGeneralMechanismLogger logger;

  /** Creates a new CharacterizeElevatorAngle. */
  public CharacterizeElevatorAngle(Subsystems subsystems) {
    this.elevatorAngle = subsystems.elevatorAngle;
    logger = new SysIdGeneralMechanismLogger(elevatorAngle::getAngle, elevatorAngle::getVelocity);

    addRequirements(elevatorAngle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    logger.init();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    logger.logData();
    elevatorAngle.setMotorVoltage(logger.getMotorVoltage());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorAngle.stopMotor();
    logger.sendData();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; //logger.getMotorVoltage() >= 0 ? elevatorAngle.atScoringLimit() : elevatorAngle.atAcquiringLimit();
  }
}
