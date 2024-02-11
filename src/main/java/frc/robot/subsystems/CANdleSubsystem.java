// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;

import edu.wpi.first.util.sendable.SendableBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */
public class CANdleSubsystem extends SubsystemBase {
    public static class CANdleConstants{
        public static int CANDLE_CAN_ID = 50;
        public static int NUMBER_OF_LED = 20;
        public static double ANIMATION_SPEED = 0.50; // Percent
        public static double LED_BRIGHTNESS = 1.00;  // Percent
        
        //NOTE: The colors of the CANdle are in GRB. REMEMBER!!!

        //Color combinations. In RGB. 
        public static int COLORS[][] = {
            {255,   0,   0},/* Red */     {255, 165,   0},/* Orange */      {255, 255,   0},/* Yellow */
            { 50, 205,  50},/* Lime */    {144, 238, 144},/* Light Green */ {  0, 255,   0},/* Green */
            {  0, 255, 255},/* Cyan */    {173, 216, 230},/* Light Blue */  {  0,   0, 255},/* Blue */
            {127,   0, 255},/* Violet */  {255,   0, 255},/* Magenta */     {255, 193, 203},/* Pink */
            {255, 255, 255},/* White */   {  0,   0,   0},/* Black */
        };
    }

    public CANdle candle = new CANdle(CANdleConstants.CANDLE_CAN_ID);

    public int mainPrimaryColor;
    public int mainSecondaryColor;
    public int mainPattern;

    public int r;
    public int g;
    public int b;

    public CANdleSubsystem() {
        //Set the CANdle to configure settings
        CANdleConfiguration config = new CANdleConfiguration();
        //Setting the strip connected to RGB
        config.stripType = LEDStripType.RGB;
        //Setting the brighness of the LED's for solid colors
        config.brightnessScalar = CANdleConstants.LED_BRIGHTNESS;
        //"Burn"(it doesn't) the settings to the CANdle
        candle.configAllSettings(config);
        //Turning off the 5V rail of the CANdle output
        candle.configV5Enabled(false);

        candle.clearStickyFaults();
        

        Shuffleboard.getTab("CANdle Testing").add(this);
    }

    /**
     * Sets the color of the LED strips connected to the CANdle
     * @param primaryColor The color to be used as the base color. The color and
     * the number they corospond to is listed:
     *    -0: Red   -1: Orange   -2: Yellow   -3: Lime   -4: Light Green   -5: Green
     *    -6: Cyan   -7: Light Blue   -8: Blue   -9: Violet   -10: Magenta   -11: Pink
     *    -12: White   -13: Black
     * @param secondaryColor The color to be used in the Twinkle pattern as the pattern
     * These colors also come from the list for primaryColor
     * @param animation The animation wanted. The pattern and the number they corespond
     * to is listed:
     *   -0: Rainbow   -1: Rainbow Fade   -2: Twinkle; Requires a primary and secondary color
     *   -3: Fade; Requires primary only   -4: Solid; Requires primary only
     */
    public void setPattern(int primaryColor, int secondaryColor, int animation){
        switch(animation){

            case 0: // Rainbow
                RainbowAnimation rainbowAnim = new RainbowAnimation(CANdleConstants.LED_BRIGHTNESS, 
                    CANdleConstants.ANIMATION_SPEED, CANdleConstants.NUMBER_OF_LED);
                candle.animate(rainbowAnim);
                break;

            case 1: // Rainbow Fade
                RgbFadeAnimation rainbowFadeAnim = new RgbFadeAnimation(CANdleConstants.LED_BRIGHTNESS, 
                    CANdleConstants.ANIMATION_SPEED, CANdleConstants.NUMBER_OF_LED);
                candle.animate(rainbowFadeAnim);
                break;

            case 2: // Sparkle with Background
                TwinkleAnimation twinkleAnim = new TwinkleAnimation(getColorR(secondaryColor), getColorG(secondaryColor),
                    getColorB(secondaryColor), 0, CANdleConstants.ANIMATION_SPEED, CANdleConstants.NUMBER_OF_LED, null);
                // Setting the background color before the pattern
                candle.setLEDs(getColorG(primaryColor), getColorR(primaryColor), getColorB(primaryColor));
                candle.animate(twinkleAnim);
                break;
                
            case 3: // Fade
                SingleFadeAnimation fadeAnim = new SingleFadeAnimation(getColorG(primaryColor), getColorR(primaryColor), 
                    getColorB(primaryColor), 0, CANdleConstants.ANIMATION_SPEED, CANdleConstants.NUMBER_OF_LED);
                candle.animate(fadeAnim);
                break;

            case 4: // Solid
                candle.setLEDs(getColorG(primaryColor), getColorR(primaryColor), getColorB(primaryColor));
                break;
            
            default:
                break;

        }
    }

    public void setPrimaryColor(long color){
        mainPrimaryColor = (int)color;
    }
    public void setSecondaryColor(long color){
        mainSecondaryColor = (int)color;
    }
    public void setPattern(long pattern){
        mainPattern = (int)pattern;
    }

    public void setR(long R){
        r = (int)R;
    }
    public void setG(long G){
        g = (int)G;
    }
    public void setB(long B){
        b = (int)B;
    }
    public void setCustomPattern(boolean start){
        if(start){
            candle.setLEDs(g,r,b);
        }else{
            candle.setLEDs(0,0,0);
        }
    }

    public void initSendable(SendableBuilder builder){
        builder.setSmartDashboardType("CANdleSubsystem");

        builder.addIntegerProperty("Set R", null, this::setR);
        builder.addIntegerProperty("Set G", null, this::setG);
        builder.addIntegerProperty("Set B", null, this::setB);
        builder.addBooleanProperty("Custom Color", null, this::setCustomPattern);
        builder.addIntegerProperty("Set Primary Color", null, this::setPrimaryColor);
        builder.addIntegerProperty("Set Secondary Color", null, this::setSecondaryColor);
        builder.addIntegerProperty("Pattern", null, this::setPattern);
    }

    private int getColorR(int color){
        return CANdleConstants.COLORS[color][0]; 
    }
    private int getColorG(int color){
        return CANdleConstants.COLORS[color][1]; 
    }
    private int getColorB(int color){
        return CANdleConstants.COLORS[color][2]; 
    }
    
    

}
