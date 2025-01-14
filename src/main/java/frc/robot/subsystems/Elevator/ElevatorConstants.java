// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Elevator;

import java.util.function.Function;

import edu.wpi.first.math.Pair;
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

        private double height;

        private ElevatorLevel(double height){
            this.height = height;
        }

        public double getHeight(){
            return this.height;
        }

        public static Pair<ElevatorLevel, Double> findNearestLevel(double number) { // I know it's not good, what's a better way?
            ElevatorLevel nearestLevel = null;
            double smallestDifference = 10;
        
            for (ElevatorLevel level : ElevatorLevel.values()) {
                double difference = Math.abs(level.getHeight() - number);
                if (difference < smallestDifference){
                    nearestLevel = level;
                    smallestDifference = difference;
                }
            }
        
            return new Pair<ElevatorLevel, Double>(nearestLevel, smallestDifference);
        }
        
    }

    public class LeadMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = null;

        public static final int MOTOR_ID = 0;
        public static final double GEAR_RATIO = 1;

        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_MAX;

        public static final double SOFT_MINIMUM = 0;
        public static final double SOFT_MAXIMUM = 0;

        public static final PIDFGains PID_GAINS = null;
        public static final Function<Double, Double> FEED_FORWARD = null;
        
        public static final boolean IS_INVERTED = false;
    }

    public class FollowMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = null;

        public static final int MOTOR_ID = 0;
        public static final double GEAR_RATIO = 1;
        
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_MAX;

        public static final double SOFT_MINIMUM = 0;
        public static final double SOFT_MAXIMUM = 0;

        public static final PIDFGains PID_GAINS = null;
        public static final Function<Double, Double> FEED_FORWARD = null;
        
        public static final boolean IS_INVERTED = false;
    }

    public static final double ELEVATOR_TOLERANCE = 0.1;
    public static final double LEVEL_TOLERANCE = 0.5;

    public static final double HEIGHT_TO_ROTATION = 1;
    public static final double ROTATION_TO_HEIGHT = 1 / HEIGHT_TO_ROTATION;

    public static final double X_ON_ROBOT = 1;
    public static final double Y_ON_ROBOT = 1;
}
