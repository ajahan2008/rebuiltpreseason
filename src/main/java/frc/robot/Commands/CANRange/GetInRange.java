// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.CANRange;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANRange;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveRequest.RobotCentric;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class GetInRange extends Command {

  private CANRange canRange;
  private CommandSwerveDrivetrain drivetrain;
  private double goalDistance;
  private RobotCentric robotCentric = new RobotCentric();
  private PIDController pidController = new PIDController(.9, 0, 0);

  /** Creates a new GetInRange. */
  public GetInRange(CANRange canRange, CommandSwerveDrivetrain drivetrain, RobotCentric robotCentric, double goalDistance) {
    this.goalDistance = goalDistance;
    this.drivetrain = drivetrain;
    this.canRange = canRange;
    // addRequirements(drivetrain);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    canRange.useCanrange();
    System.out.println("init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (canRange.getDistance() > goalDistance) {
    System.out.println("running");
    canRange.useCanrange();
    drivetrain.setControl(robotCentric.withVelocityX(pidController.calculate(1)).withVelocityY(0).withRotationalRate(0));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    canRange.inRange();
    drivetrain.setControl(robotCentric.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
    System.out.println("end");
    drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
    canRange.stopCanrange();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
