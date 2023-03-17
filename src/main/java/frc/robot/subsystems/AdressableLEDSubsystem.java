// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.ColorConstants.*;

import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.AddressableLEDs;
import frc.robot.Constants.RobotConstants.PWMPort;


public class AdressableLEDSubsystem extends SubsystemBase {
  private boolean isYellow = false;
  private AddressableLEDs leds = new AddressableLEDs(PWMPort.LED, 51);
  /** Creates a new AdressableLEDs. */
  public AdressableLEDSubsystem() {
    leds.start();
    leds.setColor(RED);
    leds.commitColor();
  }
  public void setGamePieceColor() {
    if (isYellow) {
      leds.setColor(PURPLE);
    } else {
      leds.setColor(YELLOW);
    }
    isYellow = !isYellow;
    leds.commitColor();
  }

  public boolean isYellow() {
    return isYellow;
  }

  public void setColor(Color8Bit color, int index) {
    leds.setColor(color, index);
  }

  public void commitColor() {
    leds.commitColor();
  }

  public void fillColor(Color8Bit color) {
    leds.setColor(color);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
