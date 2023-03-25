// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.ColorConstants.RED;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.LightningBoltLEDSubsystem;

public class PulseLED extends CommandBase {
  private final Color8Bit HUE1 = new Color8Bit(Color.fromHSV(28, 255, 255));
  private final Color8Bit HUE2 = new Color8Bit(Color.fromHSV(28, 192, 192));
  private final Color8Bit HUE3 = new Color8Bit(Color.fromHSV(28, 128, 128));
  private final double STEP_TIME = 0.2;
  private final int LED_COUNT = 32;
  private final int PULSE_LENGTH = 5;

  private final LightningBoltLEDSubsystem led;
  private int step; 
  private int direction = 1;
  private final Timer timer = new Timer();

  /** Creates a new PulseLED. */
  public PulseLED(LightningBoltLEDSubsystem led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    step = 0;
    direction = 1;
    paintBoltLED();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (timer.advanceIfElapsed(STEP_TIME)) {
      step += direction;
      if (direction > 0) {
        if (step >= (LED_COUNT - PULSE_LENGTH - 1)) {
          direction *= -1;
          step = LED_COUNT - PULSE_LENGTH - 1;
        }
      }
      else {
        if (step <= 0) {
          direction *= -1;
          step = 0;
        }
      }
      paintBoltLED();
    }
  }

  private void paintBoltLED() {
    led.fillColor(RED);
    led.setColor(HUE3, step);
    led.setColor(HUE2, step + 1);
    led.setColor(HUE1, step + 2);
    led.setColor(HUE2, step + 3);
    led.setColor(HUE3, step + 4);
    led.commitColor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

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
