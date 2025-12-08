// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LimelightsCombined;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleLEDs;
import frc.robot.subsystems.LimelightOne;
import frc.robot.subsystems.LimelightTwo;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightsDetection extends Command {

  private CANdleLEDs candle;
  
    /**
     * Utilizes both Limelights to search for an AprilTag.
     * @param candle - CANdle subsystem
     */
    public LimelightsDetection(CANdleLEDs candle) {
    this.candle = candle;
    addRequirements(candle);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    candle.clearAnimation(0, 399);
    System.out.println("COMMAND INIT");
    if (LimelightOne.hasValidTargets() == 1 || LimelightTwo.hasValidTargets() == 1) {
      candle.turnGreen();
      System.out.println("DETECTING!");
    } else {
      candle.turnRed();
      System.out.println("NOT DETECTING!");
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    candle.inUse();
    System.out.println("RUNNING");
    if (LimelightOne.hasValidTargets() == 1 || LimelightTwo.hasValidTargets() == 1) {
      candle.turnGreen();
      System.out.println("DETECTING!");
    } else {
      candle.turnRed();
      System.out.println("NOT DETECTING!");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("FINISHED!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
