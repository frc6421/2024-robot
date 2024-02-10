// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;

/** Add your docs here. */
public class CANdleSubsystem {
    public static class CANdleConstants{
        public static int CANDLE_CAN_ID = 0;
        public static int NUMBER_OF_LED = 0;
        
        //All of the different color combinations we COULD have and want
        public static enum colors{
            RED,                    ORANGE,                 YELLOW, //Reds
            LIME,                   YELLOW_GREEN,           GREEN,  //Greens
            LIGHT_BLUE,             CYAN,                   BLUE,   //Blues 
            VIOLET,                 MAGENTA,                PINK,   //Purples
            WHITE,
            RAINBOW_4_COLOR,        RAINBOW_8_COLOR,        RAINBOW_16_COLOR,       RAINBOW_32_COLOR,   //Rainbow of which varies in the amount of colors they display
            FADE_4_COLOR,           FADE_8_COLOR,           FADE_16_COLOR,          FADE_32_COLOR,      //Fade of which varies in the amount of colors they display
            SPARKLE_RED_ON_WHITE,   SPARKLE_BLUE_ON_WHITE,  SPARKLE_BLACK_ON_WHITE,     //Sparkle of color on primarily white
            SPARKLE_WHITE_ON_RED,   SPARKLE_WHITE_ON_BLUE,  SPARKLE_WHITE_ON_BLACK,     //Sparkle of white on primarily color
        }
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

    

}
