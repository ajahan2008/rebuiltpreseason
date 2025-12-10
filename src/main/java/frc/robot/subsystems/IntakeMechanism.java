// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeMechanism extends SubsystemBase {

  private TalonFX intakeMotor = new TalonFX(Constants.IntakeIDs.intakeMotor);
  private TalonFX pivotMotor = new TalonFX(Constants.IntakeIDs.pivotMotor);

  /** Creates a new IntakeMechanism. */
  public IntakeMechanism() {}

  public void pivotSpeed(double speed) {
    intakeMotor.set(speed);
  }

  public void orangeIntake(double speed) {
    intakeMotor.set(speed);
  }

  public void stopAll() {
    intakeMotor.stopMotor();
    pivotMotor.stopMotor();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Intake Position", intakeMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("intake pivot", pivotMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Intake Speed", intakeMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("intake pivot", pivotMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Intake Stator Voltage", intakeMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Stator Voltage", pivotMotor.getMotorVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Intake Supply Voltage", intakeMotor.getSupplyVoltage().getValueAsDouble());
    SmartDashboard.putNumber("Pivot Supply Voltage", pivotMotor.getSupplyVoltage().getValueAsDouble());

    // This method will be called once per scheduler run
  }
}
