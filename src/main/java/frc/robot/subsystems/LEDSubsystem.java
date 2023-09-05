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

  private Color Y = new Color(0.2, 0.15, 0);
  private Color P = new Color(0, 0, 0.25);

  public LEDSubsystem() {
    led.setLength(ledBuffer.getLength());
    led.setData(ledBuffer);
    led.start();

    setAllLeds(new Color(0, 0, 0.25));
  }

  public void killLeds() {
    setAllLeds(Color.kBlack);
    led.stop();
  }

  public void setBlue(){ setAllLeds(P); }

  public void setYellow(){ setAllLeds(Y); }

  public void setAllLeds(/*int r, int g, int b*/ Color color) {
    for (int i = 0; i < ledBuffer.getLength(); i++) ledBuffer.setLED(i, color);
    
    led.setData(ledBuffer);
  }
}
