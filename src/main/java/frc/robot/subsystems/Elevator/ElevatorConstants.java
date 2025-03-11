// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

/** Add your docs here. */
public class ElevatorConstants {
    public enum ElevatorLevel{
        Bottom(0.65),
        L1(0.71),
        L2(0.9),
        L3(1.28),
        L4(1.91);

        private final double height;


        ElevatorLevel(double height){
            this.height = height;
        }

        public double getHeight(){
            return this.height;
        }

        public static ElevatorLevel findNearestLevel(double height) {
            for (ElevatorLevel level : ElevatorLevel.values()) {
                double distance = Math.abs(level.getHeight() - height);
                if (distance < ElevatorConstants.ELEVATOR_TOLERANCE) return level;
            }
            return null;
        }
    }

    public static class LeadMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 17;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        
        public static final boolean IS_INVERTED = false;
    }

    public static class FollowMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 16;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;

        public static final boolean IS_INVERTED = true;
    }

    public static final double X_ON_ROBOT = 0;
    public static final double Y_ON_ROBOT = 0;
    public static final double Z_OFFSET = -0.6;

    public static final double kV = 1.7211;
    public static final double kA = 0.15653;
    public static final double ELEVATOR_TOLERANCE = 0.015;

    // public static final Constraints PROFILE = new Constraints(5, 12);
    public static final Constraints PROFILE = new Constraints(7, 12);

    public static final double ELEVATOR_WEIGHT = 1;
    public static final double PULLEY_RADIUS = 0.024;
    
    public static final double GEAR_RATIO = 5;
    public static final double PULLEY_EXTENSION_RATIO = PULLEY_RADIUS * 2 * Math.PI * 2;

    public static final double SOFT_MINIMUM = ElevatorLevel.Bottom.getHeight();
    public static final double SOFT_MAXIMUM = ElevatorLevel.L4.getHeight();

    public static final PIDFGains PID_GAINS = new PIDFGains(
        1.5,
        10,
        0,
        0,
        ELEVATOR_TOLERANCE,
        0.02);
    public static final double FEED_FORWARD = 0.34285;
    public static final double STATIC_FEEDFORWARD = 0.07;
}
