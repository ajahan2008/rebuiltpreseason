// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LimelightOne;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANdleLEDs;
import frc.robot.subsystems.LimelightOne;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LimelightDetection extends Command {

  private CANdleLEDs candle;
  
    /** Creates a new LimelightDetection. */
    public LimelightDetection(CANdleLEDs candle) {
    this.candle = candle;
    addRequirements(candle);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    System.out.println("COMMAND INIT");
    
    candle.clearAnimation(0, 399);

    if (LimelightOne.hasValidTargets() == 1) {
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
    System.out.println("RUNNING");
    if (LimelightOne.hasValidTargets() == 1) {
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
    candle.clearAnimation(0, 399);
    candle.turnViolet();
    candle.changeAnimation();
    System.out.println("FINISHED!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
