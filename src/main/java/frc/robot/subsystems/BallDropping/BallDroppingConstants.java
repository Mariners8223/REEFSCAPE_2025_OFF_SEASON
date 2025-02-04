// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class BallDroppingConstants {
    public class AngleMotor{
        //TODO: Find all constants!!!

        public static final double AngleToReachTop = 0.38; 
        public static final double AngleToReachBootom = 0.22;
        public static final double AngleToReset = 0; 

        public static final ControllerLocation location = ControllerLocation.MOTOR;
        public static final int id = 24;
        public static final boolean isBrushless = true;
        public static final double gearRatio = 9;
        public static final boolean isInverted = true;

        public static final double motorFeedForward = 0.2;
        
        public static final PIDFGains AnglePID = new PIDFGains(0.7,0,0);
    }

    public static final double AngleTolarance = 0.01;

    public class DropperMotor{
        public static final double PowerToReach = -0.8;//what power to use for the drop
        public static final int id = 20;
        public static final double maxDropperPower = 0.8;
    }
}