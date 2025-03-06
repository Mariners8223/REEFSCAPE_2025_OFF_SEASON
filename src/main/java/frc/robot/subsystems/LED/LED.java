// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Second;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class LED extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;
  LEDPattern pattern;
  /** Creates a new LED. */
  public LED() {
    led = new AddressableLED(LEDConstants.LED_PORT);
    led.setLength(LEDConstants.LED_LENGTH);
    buffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    pattern = LEDPattern.solid(Robot.isRedAlliance ? Color.kRed : Color.kBlue);
    pattern.applyTo(buffer);

    led.setData(buffer);
    led.start();

    // LEDPattern.rainbow(0, 0).scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Hertz));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pattern.applyTo(buffer);
    led.setData(buffer);
  }

  public void setSolidColour(Color colour){
    pattern = LEDPattern.solid(colour);
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
}
