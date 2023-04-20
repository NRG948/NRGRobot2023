// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IndexerSubsystem;

public class WaitForCubeShot extends CommandBase {
  private boolean cubeDetected;
  private final IndexerSubsystem indexer;
  /** Creates a new WaitForCubeShot. */
  public WaitForCubeShot(IndexerSubsystem indexer) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.indexer = indexer;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cubeDetected = indexer.isCubeDetected();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean currentCubeDetection = indexer.isCubeDetected();
    if(cubeDetected && !currentCubeDetection) {
      return true;
    }
    cubeDetected = currentCubeDetection;
    return false;
  }
}
