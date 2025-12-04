// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.Constants.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.controls.TwinkleAnimation;
import com.ctre.phoenix6.controls.TwinkleOffAnimation;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private static final double printPeriod = .5;
  private final CANBus canBus = new CANBus("rio");
  private final CANrange canrange = new CANrange(Constants.CTREDevices.canrangeID, canBus);

  private double currentTime = Timer.getFPGATimestamp();

  private static final RGBWColor white = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
  private static final RGBWColor violet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor red = RGBWColor.fromHex("#D9000000").orElseThrow();

  /*
   * Start and end index for LED animations
   * 0-8 are onboard, 8-399 are an external strip
   * CANdle supports 8 animation slots (0-7)
   */

  private static final int kSlot0StartIdx = 0;
  private static final int kSlot0EndIdx = 200;

  private static final int kSlot1StartIdx = 201;
  private static final int kSlot1EndIdx = 399;

  private final CANdle candle = new CANdle(Constants.CTREDevices.candleID, "rio");
  private final CANdi candi = new CANdi(Constants.CTREDevices.candiID, "rio");

  private enum AnimationType {
    None,
    ColorFlow,
    Fire,
    Larson,
    Rainbow,
    RgbFade,
    SingleFade,
    Strobe,
    Twinkle,
    TwinkleOff
  }

  private AnimationType m_anim0State = AnimationType.None;
  private AnimationType m_anim1State = AnimationType.None;

  private final SendableChooser<AnimationType> m_anim0Chooser = new SendableChooser<>();
  private final SendableChooser<AnimationType> m_anim1Chooser = new SendableChooser<>();

  public Robot() {
    m_robotContainer = new RobotContainer();

    CANrangeConfiguration canrangeConfig = new CANrangeConfiguration();

    canrangeConfig.ProximityParams.MinSignalStrengthForValidMeasurement = 2000; // if (2000 signal strength), valid
    canrangeConfig.ProximityParams.ProximityThreshold = 1; // if (object <= .1 meters), isDetected

    canrangeConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz; // fast refresh rate

    canrange.getConfigurator().apply(canrangeConfig);

    var candleConfig = new CANdleConfiguration();
    candleConfig.LED.StripType = StripTypeValue.GRB;
    candleConfig.LED.BrightnessScalar = 0.5;
    candleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(candleConfig);

    for (int i = 0; i < 8; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }
    candle.setControl(new SolidColor(0, 3).withColor(violet));
    candle.setControl(new SolidColor(4, 7).withColor(white));

    /* add animations to chooser for slot 0 */
    m_anim0Chooser.setDefaultOption("Color Flow", AnimationType.ColorFlow);
    m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
    m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
    m_anim0Chooser.addOption("Twinkle Off", AnimationType.TwinkleOff);
    m_anim0Chooser.addOption("Fire", AnimationType.Fire);

    /* add animations to chooser for slot 1 */
    m_anim1Chooser.setDefaultOption("Larson", AnimationType.Larson);
    m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
    m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
    m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
    m_anim1Chooser.addOption("Fire", AnimationType.Fire);

    SmartDashboard.putData("Animation 0", m_anim0Chooser);
    SmartDashboard.putData("Animation 1", m_anim1Chooser);

  }

  @Override
  public void robotPeriodic() {
    var isDetected = canrange.getIsDetected(false);
    if (Timer.getFPGATimestamp() - currentTime > printPeriod) {
      currentTime += printPeriod;
      var distance = canrange.getDistance();
      var signalStrength = canrange.getSignalStrength();
      System.out.println("Distance is " + distance.toString() + " with a signal strength of " + signalStrength + " and " + distance.getTimestamp().getLatency() + " seconds of latency.");

      isDetected.waitForUpdate(printPeriod);
      System.out.println("Is detected is " + isDetected.getValue() + "  " + isDetected.getUnits() + " with " + isDetected.getTimestamp().getLatency() + " seconds of latency.");

      SmartDashboard.putBoolean("CANRange", isDetected.getValue());
      SmartDashboard.putNumber("CANRange Distance (m)", distance.getValueAsDouble());
      SmartDashboard.putNumber("CANRange Distance (in)", distance.getValueAsDouble() * 39.38);
      SmartDashboard.putNumber("CANRange Latency", isDetected.getTimestamp().getLatency());

    if (isDetected.getValue()) {
      RGBWColor green = new RGBWColor(0, (int)(255 * (1 - (distance.getValueAsDouble() / .4))), 0);
      candle.setControl(new SolidColor(kSlot0StartIdx, kSlot1EndIdx).withColor(green));
    } else {
      candle.setControl(new SolidColor(kSlot0StartIdx, kSlot1EndIdx).withColor(red));
    }

      CommandScheduler.getInstance().run();
    }
    
    // final var anim0Selection = m_anim0Chooser.getSelected();
    // if (m_anim0State != anim0Selection) {
    //     m_anim0State = anim0Selection;

    //     switch (m_anim0State) {
    //         default:
    //         case ColorFlow:
    //             if (isDetected.getValue()) {
    //               candle.setControl(
    //                 new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                     .withColor(green)
    //             );
    //           } else {
    //             candle.setControl(
    //                 new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                     .withColor(violet)
    //             );
    //           };
    //             break;
    //         case Rainbow:
    //             candle.setControl(
    //                 new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //             );
    //             break;
    //         case Twinkle:
    //             candle.setControl(
    //                 new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                     .withColor(violet)
    //             );
    //             break;
    //         case TwinkleOff:
    //             candle.setControl(
    //                 new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //                     .withColor(violet)
    //             );
    //             break;
    //         case Fire:
    //             candle.setControl(
    //                 new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
    //             );
    //             break;
    //     }
    // }

    // /* if the selection for slot 1 changes, change animations */
    // final var anim1Selection = m_anim1Chooser.getSelected();
    // if (m_anim1State != anim1Selection) {
    //     m_anim1State = anim1Selection;

    //     switch (m_anim1State) {
    //         default:
    //         case Larson:
    //             if (isDetected.getValue()) {
    //               candle.setControl(
    //                 new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
    //                     .withColor(green)
    //             );
    //             } else {
    //             candle.setControl(
    //                 new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
    //                     .withColor(red)
    //             );}
    //             break;
    //         case RgbFade:
    //             candle.setControl(
    //                 new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
    //             );
    //             break;
    //         case SingleFade:
    //             candle.setControl(
    //                 new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
    //                     .withColor(red)
    //             );
    //             break;
    //         case Strobe:
    //             candle.setControl(
    //                 new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
    //                     .withColor(red)
    //             );
    //             break;
    //         case Fire:
    //             /* direction can be reversed by either the Direction parameter or switching start and end */
    //             candle.setControl(
    //                 new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
    //                     .withDirection(AnimationDirectionValue.Backward)
    //                     .withCooling(0.4)
    //                     .withSparking(0.5)
    //             );
    //             break;
    //     }
    // }
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}
