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
        }
        //All the different animations that the CANdle can support
        public static enum animations{
            RAINBOW,            // Goes through all colors
            SPARKLE,            // Has a background color and a color that can flicker
            FADE,               // Fades the selected color
            FADE_TO_PERCENT,    // Puts the color to a level of opacity      
            FADE_RAINBOW,       // Rainbow patter that fades
            SOLID,              // Sets a color to display
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
