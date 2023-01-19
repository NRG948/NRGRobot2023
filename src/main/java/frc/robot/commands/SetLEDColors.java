// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AddressableLEDSubsystem;

public class SetLEDColors extends CommandBase {
  /** Creates a new SetLEDColors. */
  private AddressableLEDSubsystem led;

  public SetLEDColors() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(led);
  }

  @Override
  public void initialize() {

  }
  // Called when the command is initially scheduled.
  public void initialize(AddressableLED strip, Color8Bit color) {
    led.setColor(strip, color);
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
    return false;
  }
}
