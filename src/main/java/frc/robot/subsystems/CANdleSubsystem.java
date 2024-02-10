// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

/** Add your docs here. */
public class CANdleSubsystem {
    public static class CANdleConstants{
        public static int CANDLE_CAN_ID = 0;
        public static int NUMBER_OF_LED = 0;
        public static double ANIMATION_SPEED = 1.00; // Percent
        public static double LED_BRIGHTNESS = 1.00;  // Percent
        
        //Color combinations. In RGB.
        //TODO: Get the color values for all of them
        public static int COLORS[][] = {
            {255, 0,   0},/* Red */     {0,   0,   0},/* Orange */      {0,   0,   0},/* Yellow */
            {0,   0,   0},/* Lime */    {0,   0,   0},/* Light Green */ {0, 255,   0},/* Green */
            {0,   0,   0},/* Cyan */    {0,   0,   0},/* Light Blue */  {0,   0, 255},/* Blue */
            {0,   0,   0},/* Violet */  {0,   0,   0},/* Magenta */     {0,   0,   0},/* Pink */
            {255, 255, 255},/* White */ {0,   0,   0},/* Black */
        };
    }

    public CANdle candle = new CANdle(CANdleConstants.CANDLE_CAN_ID);

    public CANdleSubsystem() {
        //Set the CANdle to configure settings
        CANdleConfiguration config = new CANdleConfiguration();
        //Setting the strip connected to RGB
        config.stripType = LEDStripType.RGB;
        //"Burn"(it doesn't) the settings to the CANdle
        candle.configAllSettings(config);

        candle.clearStickyFaults();
        //Turning off the 5V rail of the CANdle output
        candle.configV5Enabled(false);
    }


    public void setPattern(int color, int animation){
        switch(animation){

            case 0: // Rainbow
                RainbowAnimation rainbowAnim = new RainbowAnimation(CANdleConstants.LED_BRIGHTNESS, CANdleConstants.ANIMATION_SPEED, CANdleConstants.NUMBER_OF_LED);
                candle.animate(rainbowAnim);
                break;

            case 1: // Rainbow Fade
                RgbFadeAnimation rainbowFadeAnim = new RgbFadeAnimation(CANdleConstants.LED_BRIGHTNESS, CANdleConstants.ANIMATION_SPEED, CANdleConstants.NUMBER_OF_LED);
                candle.animate(rainbowFadeAnim);
                break;

            case 2: // Sparkle with Background
                TwinkleAnimation twinkleAnim = new TwinkleAnimation(setColorR(color), animation, animation, animation, color, animation, null);
            case 3: // Fade
            case 4: // Solid
        }
    }

    private int setColorR(int color){
        return CANdleConstants.COLORS[color][0]; 
    }
    private int setColorG(int color){
        return CANdleConstants.COLORS[color][1]; 
    }
    private int setColorB(int color){
        return CANdleConstants.COLORS[color][2]; 
    }
    
    

}
