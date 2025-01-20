// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;


import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

/** Add your docs here. */
public class ElevatorConstants {
    public enum ElevatorLevel{
        Bottom(0),
        L1(1),
        L2(2),
        L3(3),
        L4(4),
        Intake(5),
        Moving(-1),
        NULL(-1);

        private final double height;

        private ElevatorLevel(double height){
            this.height = height;
        }

        public double getHeight(){
            return this.height;
        }

        public static ElevatorLevel findNearestLevel(double height) { // I know it's not good, what's a better way?
            for (ElevatorLevel level : ElevatorLevel.values()) {
                double distance = Math.abs(level.getHeight() - height);
                if (distance < ElevatorConstants.ELEVATOR_TOLERANCE) return level;
            }
            return ElevatorLevel.NULL;
        }
        
    }

    public static class LeadMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = null;

        public static final int MOTOR_ID = 0;
        public static final double GEAR_RATIO = 5;

        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_MAX;

        public static final double SOFT_MINIMUM = 0;
        public static final double SOFT_MAXIMUM = 0;

        public static final PIDFGains PID_GAINS = new PIDFGains(
            20,
            0,
            0);
        
        public static final boolean IS_INVERTED = false;
    }

    public static class FollowMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = null;
        public static final int MOTOR_ID = 0;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_MAX;

        public static final boolean IS_INVERTED = false;
    }

    public static final double ELEVATOR_TOLERANCE = 0.1;
    public static final double HEIGHT_TO_ROTATION = 1;

    public static final double X_ON_ROBOT = 0;
    public static final double Y_ON_ROBOT = 0;

    public static final double FEED_FORWARD = 1;

    public static final double ELEVATOR_WEIGHT = 1;
    public static final double PULLEY_RADIUS = 0.03;
}
