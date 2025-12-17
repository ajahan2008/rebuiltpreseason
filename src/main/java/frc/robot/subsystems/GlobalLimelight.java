// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class GlobalLimelight extends SubsystemBase {

  private static boolean isAligned = false;
  
    /** Creates a new GlobalLimelight. */
    public GlobalLimelight() {
      isAligned = false;
    }
  
    public static void aligned() {
      isAligned = true;
    }
  
    public static void notAligned() {
      isAligned = false;
  }

  public static boolean isAligned() {
    return isAligned;
  }

  
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Aligned", isAligned);
    // This method will be called once per scheduler run
  }
}
