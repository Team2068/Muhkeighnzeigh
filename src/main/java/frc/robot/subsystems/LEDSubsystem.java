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
    setLeds();
  }

  public void setLeds() {
    statusLEDOne();
  }

  public void statusLEDOne () {//make this into if for example arm extends, change led color to red or something
    setAllLeds(Color.kBlue);
  }

  public void killLeds() {
    setAllLeds(Color.kBlack);
    led.stop();
  }

  public void setAllLeds(/*int r, int g, int b*/ Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) {
      ledBuffer.setLED(i, color);
    }
  }

  @Override
  public void periodic() {
    led.setData(ledBuffer);
  }
}
