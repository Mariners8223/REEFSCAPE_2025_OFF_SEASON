// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.LED;

import edu.wpi.first.wpilibj.util.Color;

/** Add your docs here. */
public class LEDConstants {
    public static final int LED_PORT = 9;

    public static final int LED_LENGTH_BACK = 51;
    public static final int LED_LENGTH_MIDDLE = 23;
    public static final int LED_LENGTH_FRONT = 52;

    public static final int LED_COUNT_TOTAL = LED_LENGTH_BACK + LED_LENGTH_FRONT + LED_LENGTH_MIDDLE;

    public static final double DEFAULT_SCROLL_SPEED = 25;

    public static final Color[] BLUE_COLORS = {new Color(191,191,255), new Color(120,120, 255),
                                              new Color(73, 73, 255), new Color(31, 31, 255), new Color(0, 0, 255)};
    public static final Color BLUE_COLOR_SINGLE = new Color(31, 31, 255);
    
    public static final Color[] RED_COLORS = {new Color(255, 136, 136), new Color(251, 90, 90), new Color(244, 48, 48),
                                              new Color(233, 23, 23), new Color(193, 0, 0)};
    public static final Color RED_COLOR_SINGLE = new Color(233, 23, 23);
}
