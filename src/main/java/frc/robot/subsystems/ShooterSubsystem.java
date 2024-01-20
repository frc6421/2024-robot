// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

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
      public static final int ANGLE_MOTOR_CAN_ID = 32;

      public static final int FLEX_RPM_PER_VOLT = 560;
    }
    //Creating a PDH object for volatge measurments
    private PowerDistribution PDH = new PowerDistribution(1, ModuleType.kRev);
    //Creating the objects for the motors and their encoders, respectivly
    private CANSparkFlex topShooter;
    private CANSparkFlex bottomShooter;

    private CANSparkMax angleShooter;

    private static RelativeEncoder topEncoder;
    private static RelativeEncoder bottomEncoder;
    
    private static RelativeEncoder angleEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    //Setting their CAN ID and the type
    topShooter = new CANSparkFlex(ShooterConstants.TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
    bottomShooter = new CANSparkFlex(ShooterConstants.BOTTOM_SHOOTER_CAN_ID, MotorType.kBrushless);
    angleShooter = new CANSparkMax(ShooterConstants.ANGLE_MOTOR_CAN_ID, MotorType.kBrushless);

    //Reseting them to defaults
    topShooter.restoreFactoryDefaults();
    bottomShooter.restoreFactoryDefaults();
    angleShooter.restoreFactoryDefaults();

    //Setting the encoders
    topEncoder = topShooter.getEncoder();
    bottomEncoder = bottomShooter.getEncoder();
    angleEncoder = angleShooter.getEncoder();

    //Setting the motors idel mode. Coast means they will be free to rotate
    topShooter.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    bottomShooter.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    angleShooter.setIdleMode(CANSparkMax.IdleMode.kBrake);


    //TODO verfiy inverts
    topShooter.setInverted(false);
    topShooter.setSmartCurrentLimit(60);


    angleShooter.setInverted(false);
    angleShooter.setSmartCurrentLimit(60);

    //TODO verify following is working properly
    bottomShooter.follow(topShooter, true);

    //Creating the Shuffleboard tab for testing
    Shuffleboard.getTab("Shooter Subsystem").add(this);
  }

  double RPM;
  /**
   * Converts the given RPM from RPM to a value between 0 and 1.
   * @param rpm The set RPM, gotten from the Shuffleboard
   */
  public void setRPM(double rpm){
    if(rpm >= 1.0){ RPM /= (getPDHVoltage() * ShooterConstants.FLEX_RPM_PER_VOLT); }
  }

  /**
   * Gets the current RPM of the motor
   * @return RPM
   */
  public double getTopMotorRPM(){
    double RPM = topEncoder.getVelocity();
    RPM *= getPDHVoltage() * ShooterConstants.FLEX_RPM_PER_VOLT;
    return RPM;
  }
  /**
   * Gets the current RPM of the motor
   * @return RPM
   */
  public double getBottomMotorRPM(){
    double RPM = bottomEncoder.getVelocity();
    RPM *= getPDHVoltage() * ShooterConstants.FLEX_RPM_PER_VOLT;
    return RPM;
  }
  /**
   * Runs the motor forward
   */
  public void runMotorForward(){
    topShooter.set(RPM);
  }
  /**
   * Runs the motor backward
   */
  public void runMotorBackward(){
    topShooter.set(-RPM);
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

    builder.addDoubleProperty("Set Motor Percent or RPM", null, this::setRPM);
    builder.addDoubleProperty("Top Motor RPM", this::getTopMotorRPM, null);
    builder.addDoubleProperty("Bottom Motor RPM", this::getBottomMotorRPM, null);
    builder.addDoubleProperty("PDH Volatage", this::getPDHVoltage, null);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
