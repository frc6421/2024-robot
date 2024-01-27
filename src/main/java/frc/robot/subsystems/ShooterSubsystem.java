// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;


/*
 * NOTE: The Angle Neo is unused!!!!! Please veryify you have stuff set up for it!
 */
public class ShooterSubsystem extends SubsystemBase {
    public static class ShooterConstants{
      public static final int TOP_SHOOTER_CAN_ID = 30;
      public static final int BOTTOM_SHOOTER_CAN_ID = 31;

      public static final int FLEX_RPM_PER_VOLT = 560;
    }
    //Creating a PDH object for volatge measurments
    private PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
    //Creating the objects for the motors and their encoders, respectivly
    private CANSparkFlex topShooter;
    private CANSparkFlex bottomShooter;


    private static RelativeEncoder topEncoder;
    private static RelativeEncoder bottomEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {

    //Setting their CAN ID and the type
    topShooter = new CANSparkFlex(ShooterConstants.TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
    bottomShooter = new CANSparkFlex(ShooterConstants.BOTTOM_SHOOTER_CAN_ID, MotorType.kBrushless);

    //Reseting them to defaults
    topShooter.restoreFactoryDefaults();
    bottomShooter.restoreFactoryDefaults();

    //Setting the encoders
    topEncoder = topShooter.getEncoder();
    bottomEncoder = bottomShooter.getEncoder();

    //Setting the motors idel mode. Coast means they will be free to rotate
    topShooter.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    bottomShooter.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    topShooter.setInverted(false);
    topShooter.setSmartCurrentLimit(60);

    bottomShooter.setInverted(true);
    bottomShooter.setSmartCurrentLimit(60);

    //Creating the Shuffleboard tab for testing
    Shuffleboard.getTab("Shooter Subsystem").add(this);
  }

  double percentOutput;

  /**
   * Converts the given RPM from RPM to a value between 0 and 1.
   * @param output The set RPM, gotten from the Shuffleboard
   */
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
   * Runs the motor forward
   */
  public void runMotorForward(){
    topShooter.set(percentOutput);
    bottomShooter.set(percentOutput);
  }
  /**
   * Runs the motor backward
   */
  public void runMotorBackward(){
    topShooter.set(-percentOutput);
    bottomShooter.set(-percentOutput);
  }
  /**
   * Gets the voltage of the PDH Bus for calculations
   * @return PDH.getVoltage();
   */
  private double getPDHVoltage(){
    return PDH.getVoltage();
  }

  public void initSendable(SendableBuilder builder){
    builder.setSmartDashboardType("ShooterSubsystem");

    builder.addDoubleProperty("Set Motor Percent", null, this::setPercentOutput);
    builder.addDoubleProperty("Top Motor RPM", this::getTopMotorRPM, null);
    builder.addDoubleProperty("Bottom Motor RPM", this::getBottomMotorRPM, null);
    builder.addDoubleProperty("PDH Voltage", this::getPDHVoltage, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
