// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


/*
 * NOTE: The Angle Neo is unused!!!!! Please veryify you have stuff set up for it!
 */
public class ShooterSubsystem extends SubsystemBase {
    public static class ShooterConstants{
      public static final int TOP_SHOOTER_CAN_ID = 30;
      public static final int BOTTOM_SHOOTER_CAN_ID = 31;
      public static final int CURRENT_LIMIT = 60;
    }
    //Creating the objects for the motors and their encoders, respectivly
    private CANSparkFlex topShooterMotor;
    private CANSparkFlex bottomShooterMotor;


    private static RelativeEncoder topEncoder;
    private static RelativeEncoder bottomEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    //Setting their CAN ID and the type
    topShooterMotor = new CANSparkFlex(ShooterConstants.TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
    bottomShooterMotor = new CANSparkFlex(ShooterConstants.BOTTOM_SHOOTER_CAN_ID, MotorType.kBrushless);

    //Reseting them to defaults
    topShooterMotor.restoreFactoryDefaults();
    bottomShooterMotor.restoreFactoryDefaults();

    //Setting the encoders
    topEncoder = topShooterMotor.getEncoder();
    bottomEncoder = bottomShooterMotor.getEncoder();

    //TODO: Add PID controller

    //Setting the motors idel mode. Coast means they will be free to rotate
    topShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    bottomShooterMotor.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    //TODO: MAKE IT A FOLLOWER!!!!!!!!!!!!!!!!!!
    topShooterMotor.setInverted(false);
    topShooterMotor.setSmartCurrentLimit(ShooterConstants.CURRENT_LIMIT);
    bottomShooterMotor.follow(topShooterMotor, true);

    //Creating the Shuffleboard tab for testing
    Shuffleboard.getTab("Shooter Subsystem").add(this);
  }

  // Used for setting the percent output of other functions
  double percentOutput;

  /**
   * Takes the percent output of the motors and to allow for other 
   * functions to use it.
   * @param output The set percent, gotten from Shuffleboard
   */
  //TODO: Change from Percent output to Velocity
  public double setPercentOutput(double output){
    percentOutput = output;
    return output;
  }

  /**
   * Gets the current RPM of the motor
   * @return rpm
   */
  public double getTopMotorRPM(){
    double rpm = topEncoder.getVelocity();
    return rpm;
  }

  /**
   * Gets the current RPM of the motor
   * @return rpm
   */
  public double getBottomMotorRPM(){
    double rpm = bottomEncoder.getVelocity();
    return rpm;
  }

  /**
   * Runs the motor forward with the desired output 
   * percent
   */
  //TODO: Change to Velocity stuff and combine Methods 
  public void runMotorForward(){
    topShooterMotor.set(percentOutput);
  }

  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("ShooterSubsystem");

    builder.addDoubleProperty("Set Motor Percent", null, this::setPercentOutput);
    builder.addDoubleProperty("Top Motor RPM", this::getTopMotorRPM, null);
    builder.addDoubleProperty("Bottom Motor RPM", this::getBottomMotorRPM, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
