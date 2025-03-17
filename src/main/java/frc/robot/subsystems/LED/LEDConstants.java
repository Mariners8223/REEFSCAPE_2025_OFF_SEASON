// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import static edu.wpi.first.units.Units.Percent;

import edu.wpi.first.units.measure.Dimensionless;
import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDConstants {
    public static final int LED_PORT = 9;

    public static final int LED_LENGTH_BACK = 51;
    public static final int LED_LENGTH_MIDDLE = 23;
    public static final int LED_LENGTH_FRONT = 52;

    public static final int LED_COUNT_TOTAL = LED_LENGTH_BACK + LED_LENGTH_FRONT + LED_LENGTH_MIDDLE;

    public static final double DEFAULT_SCROLL_SPEED = 20;

    public enum AllainceColor{
        BLUE(Color.kWhite, Color.kDarkBlue, 60, 60),
        RED(Color.kDarkRed, Color.kBlanchedAlmond, 100, 40);

        public final Color MOVING_COLOR;
        public final Color BACKGORUND_COLOR;

        public final Dimensionless COLOR_BRIGHTNESS;
        public final Dimensionless BACKGROUND_BRIGHTNESS;

        AllainceColor(Color color, Color backGround, int colorBrightness, int backgorundBrihtness){
            MOVING_COLOR = color;
            BACKGORUND_COLOR = backGround;
            COLOR_BRIGHTNESS = Percent.of(colorBrightness);
            BACKGROUND_BRIGHTNESS = Percent.of(backgorundBrihtness);
        }
    }
}
