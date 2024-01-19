// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkFlex;

public class ShooterSubsystem extends SubsystemBase {
    public static class ShooterConstants{
      public static final int TOP_SHOOTER_CAN_ID = 22;
      public static final int BOTTOM_SHOOTER_CAN_ID = 23;
    }

    private CANSparkFlex topShooter;
    private CANSparkFlex bottomShooter;

    private static RelativeEncoder topEncoder;
    private static RelativeEncoder bottomEncoder;

  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
    topShooter = new CANSparkFlex(ShooterConstants.TOP_SHOOTER_CAN_ID, MotorType.kBrushless);
    bottomShooter = new CANSparkFlex(ShooterConstants.BOTTOM_SHOOTER_CAN_ID, MotorType.kBrushless);

    topShooter.restoreFactoryDefaults();
    bottomShooter.restoreFactoryDefaults();
    
    topEncoder = topShooter.getEncoder();
    bottomEncoder = bottomShooter.getEncoder();

    topShooter.setIdleMode(CANSparkFlex.IdleMode.kCoast);
    bottomShooter.setIdleMode(CANSparkFlex.IdleMode.kCoast);

    //TODO verfiy inverts
    topShooter.setInverted(false);

    topShooter.setSmartCurrentLimit(60);

    //TODO verify following is working properly
    bottomShooter.follow(topShooter, true);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
