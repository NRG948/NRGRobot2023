// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ColorConstants.RED;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddressableLEDs;
import frc.robot.Constants.RobotConstants.PWMPort;

public class LightningBoltLEDSubsystem extends SubsystemBase {
  private AddressableLEDs leds = new AddressableLEDs(PWMPort.LightningLED, 32);
  /** Creates a new LightningBoltLEDSubsystem. */
  public LightningBoltLEDSubsystem() {
    leds.start();
    leds.setColor(RED);
    leds.commitColor();
  }

  public void setColor(Color8Bit color, int index) {
    leds.setColor(color, index);
  }

  public void fillColor(Color8Bit color) {
    leds.setColor(color);
  }

  public void commitColor() {
    leds.commitColor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
