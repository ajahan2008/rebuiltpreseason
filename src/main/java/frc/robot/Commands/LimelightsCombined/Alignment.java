// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands.LimelightsCombined;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightOne;
import frc.robot.subsystems.LimelightTwo;
import frc.robot.Constants.LL1Settings;
import frc.robot.Constants.LL2Settings;
import frc.robot.LimelightHelpers;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Alignment extends Command {

  private PIDController xController, yController, rotController;
  private CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric robotCentric;
  private double maxDriveSpeed;
  private double maxAngularRate;
  private Timer dontSeeTagTimer, stopTimer;
  private double tagID = -1;

  /** Creates a new Alignment. */
  public Alignment(CommandSwerveDrivetrain drivetrain, double maxDriveSpeed, double maxAngularRate, SwerveRequest.RobotCentric robotCentric) {
    
    xController = new PIDController(.8, 0.0, 0);
    yController = new PIDController(0.6, 0.0, 0);
    rotController = new PIDController(.048, 0, 0);
    this.drivetrain = drivetrain;
    this.maxDriveSpeed = maxDriveSpeed;
    this.maxAngularRate = maxAngularRate;
    this.robotCentric = robotCentric;

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    LLCheck();

    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    this.stopTimer = new Timer();
    this.stopTimer.start();
    tagID = LimelightHelpers.getFiducialID("limelight");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (LimelightHelpers.getTV("limelight-one") && LimelightHelpers.getFiducialID("limelight-one")  == tagID) {
      this.dontSeeTagTimer.reset();
           
      System.out.println("Aligning!");
      double[] positions = LimelightHelpers.getBotPose_TargetSpace("limelight-one");
      double xSpeed = xController.calculate(positions[2]);
      double ySpeed = -yController.calculate(positions[0]);
      double rotValue = -rotController.calculate(positions[4]);

      drivetrain.setControl(
        robotCentric
          .withVelocityX(xSpeed * (maxDriveSpeed * .75)) // forward, backwards
          .withVelocityY(ySpeed * (maxDriveSpeed / 1)) // translation
          .withRotationalRate(rotValue * (maxAngularRate / 4))
      );

      if (!rotController.atSetpoint() || !yController.atSetpoint() || !xController.atSetpoint()) {
        stopTimer.reset();
      } else if (LimelightOne.hasValidTargets() == 1 || LimelightTwo.hasValidTargets() == 1) {
        LLCheck();
      } else {
        System.out.println("NO APRIL TAG.");
        drivetrain.setControl(
          robotCentric
            .withVelocityX(0)
            .withVelocityY(0)
            .withRotationalRate(0)
        );
      }
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("ALIGNMENT DONE");
    drivetrain.setControl(
      robotCentric
        .withVelocityX(0)
        .withVelocityY(0)
        .withRotationalRate(0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    SmartDashboard.putBoolean("Done Aligning", this.dontSeeTagTimer.hasElapsed(LL1Settings.dontSeeTagTime) || stopTimer.hasElapsed(LL1Settings.poseValidationTime));
    return this.dontSeeTagTimer.hasElapsed(LL1Settings.dontSeeTagTime) || stopTimer.hasElapsed(LL1Settings.poseValidationTime);
  }

  public void LLCheck() {
    if (LimelightOne.hasValidTargets() == 1) {
      LL1Config();
    } else if (LimelightTwo.hasValidTargets() == 1) {
      LL2Config();
    }
  }

  public void LL1Config() {
    xController.setP(LL1Settings.kXPID);
    yController.setP(LL1Settings.kYPID);
    rotController.setP(LL1Settings.kRotPID);
    xController.setSetpoint(LL1Settings.kXSetpoint);
    yController.setSetpoint(LL1Settings.kYSetpoint);
    rotController.setSetpoint(LL1Settings.kRotSetpoint);
    xController.setTolerance(LL1Settings.kXTolerance);
    yController.setTolerance(LL1Settings.kYTolerance);
    rotController.setTolerance(LL1Settings.kRotTolerance);
  }

  public void LL2Config() {
    xController.setP(LL2Settings.kXPID);
    yController.setP(LL2Settings.kYPID);
    rotController.setP(LL2Settings.kRotPID);
    xController.setSetpoint(LL2Settings.kXSetpoint);
    yController.setSetpoint(LL2Settings.kYSetpoint);
    rotController.setSetpoint(LL2Settings.kRotSetpoint);
    xController.setTolerance(LL2Settings.kXTolerance);
    yController.setTolerance(LL2Settings.kYTolerance);
    rotController.setTolerance(LL2Settings.kRotTolerance);
  }

}
