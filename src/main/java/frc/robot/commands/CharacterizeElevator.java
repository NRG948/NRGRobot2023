// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.nrg948.autonomous.AutonomousCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Subsystems;
import frc.robot.sysid.SysIdGeneralMechanismLogger;

@AutonomousCommand(name = "[SYSID] Characterize Elevator")
public class CharacterizeElevator extends CommandBase {
  private ElevatorSubsystem elevator;
  private SysIdGeneralMechanismLogger logger;

  /** Creates a new CharacterizeElevator. */
  public CharacterizeElevator(Subsystems subsystems) {
    this.elevator = subsystems.elevator;
    logger = new SysIdGeneralMechanismLogger(elevator::getPosition, elevator::getVelocity);

    addRequirements(elevator);
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
    elevator.setMotorVoltage(logger.getMotorVoltage());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return logger.getMotorVoltage() > 0 ? elevator.atTopLimit() : elevator.atBottomLimit();
  }
}
