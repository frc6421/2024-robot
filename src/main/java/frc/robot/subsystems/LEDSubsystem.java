// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.LEDSubsystem.LEDConstants.LEDColors;

public class LEDSubsystem extends SubsystemBase {

  public static class LEDConstants {

    public static int NUMBER_OF_LEDS = 100;

    public static enum LEDColors{
      OFF,
      HOT_PINK,
      BLUE,
      PURPLE,
      YELLOW,
      GREEN,
      RED
    }

  }

  private static AddressableLED led;

  private static AddressableLEDBuffer ledBuffer;

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
  }
  /**
   * Setting the LED strips to a color, in GRB!!!!
   * @param color the color to set them to
   */
  public static void setColor(LEDColors color) {
      switch(color){

        case HOT_PINK:
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,255,70,0);
          }
          break;
        
        case GREEN:
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,0,0,255);
          }
          break;
        
        case BLUE:
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,0, 255, 0);
          }
          break;

        case PURPLE:
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,70,180,0);
          }
          break;
        
        case YELLOW:
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,255,0,75);
          }
          break;
        
        case RED:
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,255,0,0);
          }
          break;

        case OFF:
          for (int i = 0; i < ledBuffer.getLength(); i++) {
            ledBuffer.setRGB(i,0,0,0);
          }
          break;
      }
    led.setData(ledBuffer);
  }
}

