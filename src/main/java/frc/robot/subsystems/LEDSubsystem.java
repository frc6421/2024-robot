// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.TimeUnit;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;

public class LEDSubsystem extends SubsystemBase {

  public static class LEDConstants {
    public static int NUMBER_OF_LEDS = 10;

    public static enum LEDColors{
      OFF,
      BLUE,
      HOT_PINK,
      //RAINBOW
    }

  }

  public AddressableLED led;

  public AddressableLEDBuffer ledBuffer;

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() 
  {
    led = new AddressableLED(0);
    ledBuffer = new AddressableLEDBuffer(LEDConstants.NUMBER_OF_LEDS);

    led.setLength(LEDConstants.NUMBER_OF_LEDS);
    led.setData(ledBuffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  /**
   * Setting the LED strips to a color, in GRB!!!!
   * @param color the color to set them to
   */
  public void setColor(LEDColors color) {
      for(int i = 0; i < LEDConstants.NUMBER_OF_LEDS; i++) {
        switch(color){
          case BLUE:
            ledBuffer.setRGB(i,0,0,255);
            break;

          case HOT_PINK:
            ledBuffer.setRGB(i,0,255,70);
            break;

          case OFF:
            ledBuffer.setRGB(i,0,0,0);
            break;
        }
      }
    led.setData(ledBuffer);
  }
}

