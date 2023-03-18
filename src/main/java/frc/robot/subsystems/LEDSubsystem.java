// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants;

public class LEDSubsystem extends SubsystemBase {
  /** Creates a new LEDSubsystem. */
  AddressableLED led = new AddressableLED(LEDConstants.LED_PORT);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

  public LEDSubsystem() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();
  }

  public void setLeds() {
    statusLEDOne();
    led.setData(ledBuffer);
  }

  public void statusLEDOne () {//make this into if for example arm extends, change led color to red or something
    if (true) {
      //setAllLeds(Color.kRed);
    }
  }

  public void setAllLeds(/*int r, int g, int b*/ Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setLeds();
  }
}
