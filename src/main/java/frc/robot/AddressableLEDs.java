// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;

public class AddressableLEDs {
  //private static final int NUMBER_OF_LEDS = 10; // number of LED's on the Strip
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

  public void start(){
    led.start();
  }

  public void stop(){
    led.stop();
  }

  public void setColor(Color8Bit color) {
    for(var i = 0; i<ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, color.red, color.green, color.blue);
    }
    led.setData(ledBuffer);
  }
}
