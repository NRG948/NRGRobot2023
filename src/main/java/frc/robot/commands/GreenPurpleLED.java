// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ColorConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.AdressableLEDSubsystem;

public class GreenPurpleLED extends CommandBase {
  private final AdressableLEDSubsystem led;

  /** Creates a new GreenPurpleLED. */
  public GreenPurpleLED(AdressableLEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    for (int i = 0; i < 18; i++) {
        led.setColor(ColorConstants.GREEN, i);
    }
    for (int i = 18; i < 35; i++) {
        led.setColor(ColorConstants.PURPLE, i);
    }
    for (int i = 35; i < 42; i++) {
        led.setColor(ColorConstants.GREEN, i);
    }
    for (int i = 42; i < 49; i++) {
        led.setColor(ColorConstants.PURPLE, i);
    }
    for (int i = 49; i < 58; i++) {
        led.setColor(ColorConstants.PURPLE, i);
    }
    for (int i = 58; i < RobotConstants.LED_COUNT; i++) {
        led.setColor(ColorConstants.GREEN, i);
    }
    led.commitColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    }

  @Override
  public boolean runsWhenDisabled() {
    }
}
