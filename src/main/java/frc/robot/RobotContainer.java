// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Commands.CandleClear;
import frc.robot.Commands.Intake.IntakeCommand;
import frc.robot.Commands.Intake.IntakePivotCommand;
import frc.robot.Commands.LimelightsCombined.LimelightsDetection;
import frc.robot.Constants.IntakeIDs;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CANdleLEDs;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IntakeMechanism;
import frc.robot.subsystems.LimelightOne;
import frc.robot.subsystems.LimelightTwo;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(.5).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    
    public static final LimelightOne limelightOne = new LimelightOne();
    public static final LimelightTwo limelightTWo = new LimelightTwo();

    //public static final CANdleLEDs candle = new CANdleLEDs();

    public static final IntakeMechanism intake = new IntakeMechanism();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }
 
    public double LL1HasValidTargets() {
        return LimelightOne.hasValidTargets();
    }
    public double LL2HasValidTargets() {
        return LimelightTwo.hasValidTargets();
    }

    // public void enabledLEDS() {
    //     candle.enabledIdle();
    // }

    // public void disabledLEDS() {
    //     candle.disabledIdle();
    // }

    private void configureBindings() {

        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> {

            double kSpeed = 1;
            double kRotationRate = 1;
    
            double velocityX = joystick.getLeftY() * kSpeed;
            double velocityY = joystick.getLeftX() * kSpeed;
            double rotationalRate = joystick.getRightX() * kRotationRate;

            return drive
                    .withVelocityX(-velocityX * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-velocityY * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-rotationalRate * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        // joystick.x().whileTrue(new LimelightsDetection(candle).andThen(new CandleClear(candle)));
        // joystick.y().onTrue(new CandleClear(candle));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.povUp().whileTrue(new IntakePivotCommand(intake, .5));
        joystick.povDown().whileTrue(new IntakePivotCommand(intake, -0.5));
        joystick.povRight().onTrue(new IntakeCommand(intake, 0.5).withTimeout(1));
        joystick.povLeft().onTrue(new IntakeCommand(intake, -0.5).withTimeout(1));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        drivetrain.runOnce(() -> drivetrain.seedFieldCentric());
        return autoChooser.getSelected();
    }
}
