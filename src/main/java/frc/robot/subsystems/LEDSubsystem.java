// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {

  public static class LEDConstants {
    public static int NUMBER_OF_LEDS = 60;
  }

  public AddressableLED led;

  public AddressableLEDBuffer ledBuffer;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() 
  {
    led = new AddressableLED(1);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LEDS);

    led.setLength(LEDConstants.NUMBER_OF_LEDS);
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setColor(int g, int r, int b) {
      for(int i = 0; i < LEDConstants.NUMBER_OF_LEDS; i++) {
        // GRB
        ledBuffer.setRGB(i, g, r, b);
      }
      led.setData(ledBuffer);
  }
}
