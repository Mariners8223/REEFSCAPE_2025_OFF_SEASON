// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.AddressableLEDBufferView;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;
  StripControl controlType = StripControl.TOGETHER;

  AddressableLEDBufferView bufferFront;
  AddressableLEDBufferView bufferMiddle;
  AddressableLEDBufferView bufferBack;

  LEDPattern defaultPattern;
  LEDPattern middlePattern;
  LEDPattern sidePattern;
  LEDPattern pattern;

  public enum StripControl{
    TOGETHER,
    MIDDLE,
    SIDES
  }

  /** Creates a new LED. */
  public LED() {
    led = new AddressableLED(LEDConstants.LED_PORT);
    led.setLength(LEDConstants.LED_COUNT_TOTAL);

    buffer = new AddressableLEDBuffer(LEDConstants.LED_COUNT_TOTAL);
    
    bufferFront = buffer.createView(0, LEDConstants.LED_LENGTH_FRONT-2);
    bufferMiddle = buffer.createView(LEDConstants.LED_LENGTH_FRONT - 1, LEDConstants.LED_LENGTH_FRONT + LEDConstants.LED_LENGTH_MIDDLE -2);
    bufferBack = buffer.createView(LEDConstants.LED_LENGTH_FRONT + LEDConstants.LED_LENGTH_MIDDLE - 1, LEDConstants.LED_COUNT_TOTAL - 1);

    pattern = LEDPattern.kOff;
    pattern.applyTo(buffer);

    middlePattern = pattern;
    sidePattern = pattern;

    led.setData(buffer);
    led.start();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    switch (controlType){
      case SIDES:
        sidePattern = pattern;
        break;
      case MIDDLE:
        middlePattern = pattern;
        break;
      default:
        pattern.applyTo(buffer);
    }

    if (controlType != StripControl.TOGETHER){
      sidePattern.applyTo(bufferFront);
      sidePattern.reversed().applyTo(bufferBack);
      middlePattern.applyTo(bufferMiddle);
    }

    led.setData(buffer);
  }

  public void setStripControl(StripControl control){
    controlType = control;
  }

  public InstantCommand setStripControlCommand(StripControl control){
    return new InstantCommand(() -> setStripControl(control));
  }

  public void setDefaultPattern(boolean isRedAlliance){
    defaultPattern = LEDPattern.gradient(GradientType.kContinuous, isRedAlliance ? LEDConstants.RED_COLORS : LEDConstants.BLUE_COLORS)
                    .scrollAtRelativeSpeed(Percent.of(LEDConstants.DEFAULT_SCROLL_SPEED).per(Second));
  }

  public void putDefaultPattern(){
    pattern = defaultPattern;
  }

  public InstantCommand putDefaultPatternCommand(){
    return new InstantCommand(() -> putDefaultPattern());
  }

  public void setSolidColour(Color colour){
    pattern = LEDPattern.solid(colour);
  }

  public InstantCommand SetSolidColourCommand(Color colour){
    return new InstantCommand(() -> setSolidColour(colour));
  }

  public void Blink(double seconds){
    pattern = pattern.blink(Time.ofBaseUnits(seconds, Second));
  }

  public void Blink(double onSeconds, double offSeconds){
    pattern = pattern.blink(Time.ofBaseUnits(offSeconds, Second), Time.ofBaseUnits(offSeconds, Second));
  }

  public InstantCommand BlinkCommand(double seconds){
    return new InstantCommand(() -> Blink(seconds));
  }

  public InstantCommand BlinkCommand(double onSeconds, double offSeconds){
    return new InstantCommand(() -> Blink(onSeconds, offSeconds));
  }

  public void setGradient(Color... colours){
    pattern = LEDPattern.gradient(GradientType.kDiscontinuous, colours);
  }

  public void setMovingGradient(int speedPercent, Color... colours){
    pattern = LEDPattern.gradient(GradientType.kContinuous, colours).scrollAtRelativeSpeed(Percent.per(Second).of(speedPercent));
  }

  public void setRainbow(){
    pattern = LEDPattern.rainbow(255, 255);
  }

  public void setMovingRainbow(int speedPercent){
    pattern = LEDPattern.rainbow(256, 256).scrollAtRelativeSpeed(Percent.per(Second).of(speedPercent));
  }

  public void setProgressLayer(DoubleSupplier percent){
    pattern = pattern.mask(LEDPattern.progressMaskLayer(percent));
  }

  public InstantCommand setProgressLayerCOmmand(DoubleSupplier percent){
    return new InstantCommand(() -> setProgressLayer(percent));
  }
}
