// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANdleConfiguration;
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
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.AnimationDirectionValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class CANdleLEDs extends SubsystemBase {

  private final CANBus canBus = new CANBus("rio");

  // devices
  private final CANdle candle = new CANdle(Constants.CTREDevices.candleID, canBus);

  // colors
  private static final RGBWColor white = new RGBWColor(Color.kWhite).scaleBrightness(0.5);
  private static final RGBWColor violet = RGBWColor.fromHSV(Degrees.of(270), 0.9, 0.8);
  private static final RGBWColor red = RGBWColor.fromHex("#D9000000").orElseThrow();
  private static final RGBWColor green = new RGBWColor(0, 255, 0);
  private static final RGBWColor turquoise = new RGBWColor(0, 255, 255);
  private static final RGBWColor orange = new RGBWColor(229, 83, 0 );
  


  /*slots and indexes. 
  0-7 animation slots. 
  Idx 0-8 are onboard (on the candle), 
  8-399 are external strip.
  */
  private static final int kSlot0StartIdx = 0;
  private static final int kSlot0EndIdx = 200;
  private static final int kSlot1StartIdx = 201;
  private static final int kSlot1EndIdx = 399;

  // Animation types/states
  private enum AnimationType {
    None,
    SolidColor,
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

  private boolean commandUsing = false;


  /** Creates a new CANdle. */
  public CANdleLEDs() {

    notInUse();
    commandUsing = false;

    var candleConfig = new CANdleConfiguration();
    candleConfig.LED.StripType = StripTypeValue.GRB;
    candleConfig.LED.BrightnessScalar = 1;
    candleConfig.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Disabled;

    candle.getConfigurator().apply(candleConfig);

    for (int i = 0; i < 8; ++i) {
      candle.setControl(new SolidColor(0, 3).withColor(red));
      candle.setControl(new SolidColor(4, 7).withColor(white));
    }

    // add animations to chooser for slot 0
    m_anim0Chooser.setDefaultOption("SolidColor", AnimationType.SolidColor);
    m_anim0Chooser.addOption("Color Flow", AnimationType.ColorFlow);
    m_anim0Chooser.addOption("Rainbow", AnimationType.Rainbow);
    m_anim0Chooser.addOption("Twinkle", AnimationType.Twinkle);
    m_anim0Chooser.addOption("Twinkle OFf", AnimationType.TwinkleOff);
    m_anim0Chooser.addOption("Fire", AnimationType.Fire);
    m_anim0Chooser.addOption("Larson", AnimationType.Larson);
    m_anim0Chooser.addOption("RGB Fade", AnimationType.RgbFade);
    m_anim0Chooser.addOption("Single Fade", AnimationType.SingleFade);
    m_anim0Chooser.addOption("Strobe", AnimationType.Strobe);
    m_anim0Chooser.addOption("None", AnimationType.None);

    // add animations to chooser for slot 1
    m_anim0Chooser.setDefaultOption("SolidColor", AnimationType.SolidColor);
    m_anim1Chooser.addOption("Color Flow", AnimationType.ColorFlow);
    m_anim1Chooser.addOption("Rainbow", AnimationType.Rainbow);
    m_anim1Chooser.addOption("Twinkle", AnimationType.Twinkle);
    m_anim1Chooser.addOption("Twinkle OFf", AnimationType.TwinkleOff);
    m_anim1Chooser.addOption("Fire", AnimationType.Fire);
    m_anim1Chooser.addOption("Larson", AnimationType.Larson);
    m_anim1Chooser.addOption("RGB Fade", AnimationType.RgbFade);
    m_anim1Chooser.addOption("Single Fade", AnimationType.SingleFade);
    m_anim1Chooser.addOption("Strobe", AnimationType.Strobe);
    m_anim0Chooser.addOption("None", AnimationType.None);

    SmartDashboard.putData("Animation 0", m_anim0Chooser);
    SmartDashboard.putData("Animation 1", m_anim1Chooser);

    changeAnimation();
    
  }

  public void changeAnimation() {

    commandUsing = false;
    System.out.println("CHANGING ANIMATION");

    final var anim0Selection = m_anim0Chooser.getSelected();
    if (m_anim0State != anim0Selection || commandUsing == false) {
        m_anim0State = anim0Selection;

        switch (m_anim0State) {
            default:
            case SolidColor:
                candle.setControl(
                  new SolidColor(kSlot0StartIdx, kSlot0EndIdx).withColor(orange)
                );
                break;
            case ColorFlow:
                candle.setControl(
                    new ColorFlowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                        .withColor(violet)
                );
                break;
            case Rainbow:
                candle.setControl(
                    new RainbowAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                    .withDirection(AnimationDirectionValue.Backward)
                );
                break;
            case Twinkle:
                candle.setControl(
                    new TwinkleAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                        .withColor(violet)
                );
                break;
            case TwinkleOff:
                candle.setControl(
                    new TwinkleOffAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                        .withColor(violet)
                );
                break;
            case Fire:
                candle.setControl(
                    new FireAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                );
                break;
            case Larson:
                candle.setControl(
                    new LarsonAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                        .withColor(red)
                );
                break;
            case RgbFade:
                candle.setControl(
                    new RgbFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                );
                break;
            case SingleFade:
                candle.setControl(
                    new SingleFadeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                        .withColor(turquoise)
                );
                break;
            case Strobe:
                candle.setControl(
                    new StrobeAnimation(kSlot0StartIdx, kSlot0EndIdx).withSlot(0)
                        .withColor(red)
                );
                break;
            case None:
                clearAnimation(0, 200);
                break;
        }
    }

    final var anim1Selection = m_anim1Chooser.getSelected();
    if (m_anim1State != anim1Selection || commandUsing == false) {
      m_anim1State = anim1Selection;
      commandUsing = false;

      switch (m_anim1State) {
          default:
          case SolidColor:
              candle.setControl(
                new SolidColor(kSlot1StartIdx, kSlot1EndIdx).withColor(orange)
              );
              break;
          case ColorFlow:
              candle.setControl(
                  new ColorFlowAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                      .withColor(violet)
              );
              break;
          case Rainbow:
              candle.setControl(
                  new RainbowAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                    .withDirection(AnimationDirectionValue.Backward)
              );
              break;
          case Twinkle:
              candle.setControl(
                  new TwinkleAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                      .withColor(violet)
              );
              break;
          case TwinkleOff:
              candle.setControl(
                  new TwinkleOffAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                      .withColor(violet)
              );
              break;
          case Fire:
              candle.setControl(
                  new FireAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
              );
              break;
              case Larson:
              candle.setControl(
                  new LarsonAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                      .withColor(red)
              );
              break;
          case RgbFade:
              candle.setControl(
                  new RgbFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
              );
              break;
          case SingleFade:
              candle.setControl(
                  new SingleFadeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                      .withColor(turquoise)
              );
              break;
          case Strobe:
              candle.setControl(
                  new StrobeAnimation(kSlot1StartIdx, kSlot1EndIdx).withSlot(1)
                      .withColor(red)
              );
              break;
          case None:
              clearAnimation(201, 399);
                break;
      }
  }

  }

  public void turnGreen() {
    commandUsing = true;
    candle.setControl(new SolidColor(kSlot0StartIdx, kSlot1EndIdx).withColor(green));
  }
  
  public void turnRed() {
    commandUsing = true;
    candle.setControl(new SolidColor(kSlot0StartIdx, kSlot1EndIdx).withColor(red));
  }

  public void turnViolet() {
    commandUsing = true;
    candle.setControl(new SolidColor(kSlot0StartIdx, kSlot1EndIdx).withColor(violet));
  }

  public void clearAnimation(int startIndex, int endIndex) {
    commandUsing = true;
    for (int i = startIndex; i < endIndex; ++i) {
      candle.setControl(new EmptyAnimation(i));
    }
    commandUsing = false;
  }

  public void inUse() {
    commandUsing = true;
  }

  public void notInUse() {
    commandUsing = false;
  }

  public boolean beingUsed() {
    return commandUsing;
  }

  public void enabledIdle() {

    clearAnimation(kSlot0StartIdx, kSlot1EndIdx);
    commandUsing = false;
    candle.setControl( new SingleFadeAnimation(kSlot0StartIdx, kSlot1EndIdx).withColor(orange));

  }

  public void disabledIdle() {
    clearAnimation(kSlot0StartIdx, kSlot1EndIdx);
    commandUsing = false;
    candle.setControl( new SolidColor(kSlot0StartIdx, kSlot1EndIdx).withColor(orange));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("Command Using", commandUsing);

  }
}
