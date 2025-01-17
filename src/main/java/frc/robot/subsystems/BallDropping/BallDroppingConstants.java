// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.BallDropping;

import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class BallDroppingConstants {
    public class AngleMotor{
        //TODO: Find all constants!!!

        public static final double AngleToReach = 90; 
        public static final double AngleToReset = 0; 

        public static final ControllerLocation location = null;
        public static final int id = 0;
        public static final boolean isBrushless = true;
        public static final PIDFGains PID_gains = null;
        public static final double gearRatio = 1;
        
        public static final PIDFGains AnglePID = new PIDFGains(0,0,0);
    }

    public class DropperMotor{
        public static final double PowerToReach = 1;//what power to use for the drop
        public static final int id = 0;
        public static final double maxDropperPower = 0.8;
    }
}