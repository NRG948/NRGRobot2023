// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.AddressableLED;

public class AddressableLEDSubsystem extends SubsystemBase {
  /** Creates a new AddressableLED. */
  private static final int NUMBER_OF_LEDS = 10; // number of LED's on the Strip
  private static AddressableLED led; // Creates the new object, on port 0
  private static final AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(NUMBER_OF_LEDS);

  public AddressableLEDSubsystem(int port) {
    led = new AddressableLED(port);
    led.setLength(NUMBER_OF_LEDS);
    
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

  public void setColor(AddressableLED led, Color8Bit color) {
    for(var i = 0; i<ledBuffer.getLength(); i++){
      ledBuffer.setRGB(i, color.blue, color.green, color.red);
    }
    led.setData(ledBuffer);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
