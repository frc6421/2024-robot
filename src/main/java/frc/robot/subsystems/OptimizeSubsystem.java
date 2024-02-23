// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OptimizeSubsystem extends SubsystemBase {
  /** Creates a new OptimizeSubsystem. */
  public OptimizeSubsystem() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void bottomShooterOptimize(ShooterSubsystem shooterSubsystem) {
    shooterSubsystem.bottomShooterMotor.getAncillaryDeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getDeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_BootDuringEnable().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_BridgeBrownout().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_DeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_ForwardHardLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_ForwardSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_FusedSensorOutOfSync().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_Hardware().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_MissingDifferentialFX().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_OverSupplyV().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_ProcTemp().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_RemoteSensorDataInvalid().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_RemoteSensorPosOverflow().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_RemoteSensorReset().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_ReverseHardLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_ReverseSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_StatorCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_SupplyCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_Undervoltage().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_UnlicensedFeatureInUse().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_UnstableSupplyV().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFault_UsingFusedCANcoderWhileUnlicensed().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getFaultField().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getIsProLicensed().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getMotionMagicIsRunning().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getProcessorTemp().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_BootDuringEnable().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_BridgeBrownout().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_DeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_ForwardHardLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_ForwardSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_FusedSensorOutOfSync().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_Hardware().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_MissingDifferentialFX().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_OverSupplyV().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_ProcTemp().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_RemoteSensorDataInvalid().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_RemoteSensorPosOverflow().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_RemoteSensorReset().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_ReverseHardLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_ReverseSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_StatorCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_SupplyCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_Undervoltage().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_UnlicensedFeatureInUse().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_UnstableSupplyV().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFault_UsingFusedCANcoderWhileUnlicensed().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getStickyFaultField().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getVersion().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getVersionBugfix().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getVersionBuild().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getVersionMajor().setUpdateFrequency(0);
    shooterSubsystem.bottomShooterMotor.getVersionMinor().setUpdateFrequency(0);
  }

  public void topShooterOptimize(ShooterSubsystem shooterSubsystem) {
    shooterSubsystem.topShooterMotor.getAncillaryDeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getDeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_BootDuringEnable().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_BridgeBrownout().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_DeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_ForwardHardLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_ForwardSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_FusedSensorOutOfSync().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_Hardware().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_MissingDifferentialFX().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_OverSupplyV().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_ProcTemp().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_RemoteSensorDataInvalid().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_RemoteSensorPosOverflow().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_RemoteSensorReset().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_ReverseHardLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_ReverseSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_StatorCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_SupplyCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_Undervoltage().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_UnlicensedFeatureInUse().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_UnstableSupplyV().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFault_UsingFusedCANcoderWhileUnlicensed().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getFaultField().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getIsProLicensed().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getMotionMagicIsRunning().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getProcessorTemp().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_BootDuringEnable().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_BridgeBrownout().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_DeviceTemp().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_ForwardHardLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_ForwardSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_FusedSensorOutOfSync().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_Hardware().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_MissingDifferentialFX().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_OverSupplyV().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_ProcTemp().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_RemoteSensorDataInvalid().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_RemoteSensorPosOverflow().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_RemoteSensorReset().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_ReverseHardLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_ReverseSoftLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_StatorCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_SupplyCurrLimit().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_Undervoltage().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_UnlicensedFeatureInUse().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_UnstableSupplyV().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFault_UsingFusedCANcoderWhileUnlicensed().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getStickyFaultField().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getVersion().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getVersionBugfix().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getVersionBuild().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getVersionMajor().setUpdateFrequency(0);
    shooterSubsystem.topShooterMotor.getVersionMinor().setUpdateFrequency(0);
  }
}
