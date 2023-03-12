// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class AddressableLEDs {
  // private static final int NUMBER_OF_LEDS = 10; // number of LED's on the Strip
  private AddressableLED led; // Creates the new object, on port 0
  private final AddressableLEDBuffer ledBuffer;


  /** Creates a new AddressableLED. */
  public AddressableLEDs(int port, int numberOfLEDs) {
    led = new AddressableLED(port);
    led.setLength(numberOfLEDs);
    ledBuffer = new AddressableLEDBuffer(numberOfLEDs);
    led.setData(ledBuffer);
  }

  public AddressableLED getLED() {
    return led;
  }

  public void start() {
    led.start();
  }

  public void stop() {
    led.stop();
  }

  public void setColor(Color8Bit color) {
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setRGB(i, color.red, color.green, color.blue);
    }
  }
  
  public void setColor(Color8Bit color, int index) {
    ledBuffer.setRGB(index, color.red, color.green, color.blue);
  }

  public void commitColor() {
    led.setData(ledBuffer);
  }

 


  /*
   * WIP Alternating Color method
   * public void alternateColor(Color8Bit color1, Color8Bit color2) {
   * 
   * for (int i = 0; i < ledBuffer.getLength(); i++) {
   * if (i % 2 == 0) {
   * ledBuffer.setRGB(i+count, color1.red, color1.green, color1.blue);
   * } else {
   * ledBuffer.setRGB(i+count, color2.red, color2.green, color2.blue);
   * }
   * }
   * }
   * public void updateAlternateColor(Color8Bit color1, Color8Bit color2) {
   * for (int i = 0; i < ledBuffer.getLength(); i++) {
   * if (ledBuffer.getLED8Bit(0) == color1) {
   * 
   * }
   * }
   * }
   */
}
