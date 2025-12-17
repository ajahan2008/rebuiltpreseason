// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class CANRange extends SubsystemBase {

  private static final double printPeriod = 1;
  private final CANBus kCanBus = new CANBus("rio");
  private CANrange canRange = new CANrange(Constants.CTREDevices.canrange, kCanBus);
  private boolean inRange = false;
  private boolean inUse;

  /** Creates a new CANRange. */
  public CANRange() {

    inUse = false;

    CANrangeConfiguration config = new CANrangeConfiguration();
    config.ProximityParams.MinSignalStrengthForValidMeasurement = 1500;
    config.ProximityParams.ProximityThreshold = 0.1;
    config.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    canRange.getConfigurator().apply(config);

  }

  public double getDistance() {
    return canRange.getDistance().getValueAsDouble();
  }

  public boolean isDetected() {
    return canRange.getIsDetected().getValue();
  }

  public void inRange() {
    inRange = true;
  }

  public void outOfrange() {
    inRange = false;
  }

  public boolean isInRange() {
    return inRange;
  }

  public void useCanrange() {
    inUse = true;
  }

  public void stopCanrange() {
    inUse = false;
  }

  @Override
  public void periodic() {

    if (inUse) {
      var distance = canRange.getDistance();
      var signalStrength = canRange.getSignalStrength();
      var isDetected = canRange.getIsDetected(false);
      isDetected.waitForUpdate(printPeriod);

      SmartDashboard.putNumber("CANRange Distance", distance.getValueAsDouble());
      SmartDashboard.putNumber("CANRange Signal Strength", signalStrength.getValueAsDouble());
      SmartDashboard.putBoolean("CANRange Detects", isDetected.getValue());
      SmartDashboard.putBoolean("CANRange in range", inRange);
    }

    // This method will be called once per scheduler run
  }
}
