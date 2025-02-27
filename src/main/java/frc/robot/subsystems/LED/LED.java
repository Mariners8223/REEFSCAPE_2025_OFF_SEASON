// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Hertz;

import edu.wpi.first.units.FrequencyUnit;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.LEDPattern.GradientType;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
  AddressableLED led;
  AddressableLEDBuffer buffer;
  /** Creates a new LED. */
  public LED() {
    led = new AddressableLED(LEDConstants.LED_PORT);
    led.setLength(LEDConstants.LED_LENGTH);
    buffer = new AddressableLEDBuffer(LEDConstants.LED_LENGTH);

    led.setData(buffer);
    led.start();

    // LEDPattern.rainbow(0, 0).scrollAtRelativeSpeed(Frequency.ofBaseUnits(1, Hertz));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    led.setData(buffer);
  }

  public void setSolidColour(Color colour){
    LEDPattern.solid(colour).applyTo(buffer);
  }

  public void setGradient(Color... colours){
    LEDPattern.gradient(GradientType.kDiscontinuous, colours);
  }

  public void setRainbow(){
    LEDPattern.rainbow(255, 255);
  }
}
