// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ColorConstants;
import frc.robot.subsystems.AdressableLEDSubsystem;

public class FlameCycle extends CommandBase {
  private final AdressableLEDSubsystem led;
  private final int redDiff = ColorConstants.RED.red - ColorConstants.YELLOW.red;
  private final int greenDiff = ColorConstants.RED.green - ColorConstants.YELLOW.green;
  private final int blueDiff = ColorConstants.RED.blue - ColorConstants.YELLOW.blue;

  private int step;

  /** Creates a new FlameCycle. */
  public FlameCycle(AdressableLEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
    led.fillColor(ColorConstants.BLACK);
    led.commitColor();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double multiplier = Math.abs(Math.sin(step++ * Math.toRadians(6)));
    Color8Bit color0 = new Color8Bit((int) (redDiff * multiplier + ColorConstants.RED.red),
        (int) (greenDiff * multiplier * ColorConstants.RED.green),
        (int) (blueDiff * multiplier * ColorConstants.RED.blue));
    Color8Bit color1 = new Color8Bit((int) (redDiff * multiplier + ColorConstants.YELLOW.red),
        (int) (greenDiff * multiplier * ColorConstants.YELLOW.green),
        (int) (blueDiff * multiplier * ColorConstants.YELLOW.blue));
    for (int i = 0; i < 63; i++) {
      Color8Bit color = (i % 2) == 0 ? color0 : color1;
      led.setColor(color, i);
    }
    led.commitColor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
