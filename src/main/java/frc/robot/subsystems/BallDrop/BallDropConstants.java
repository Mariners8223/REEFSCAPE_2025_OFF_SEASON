package frc.robot.subsystems.BallDrop;

import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

public class BallDropConstants {
    public class ArmMotor 
    {

    public static final double ANGLE_TO_REACH_TOP = 0.4;
    public static final double ANGLE_TO_REACH_LOW = 0.25;
    public static final double ANGLE_TO_RESET = 0;
    public static final double ANGLE_TOLERANCE = 0.03;
    public static final double POWER_TO_REACH = 0.8;

    public static final double TIME_TO_STAY_UP = 5;
    
    public static final double SOFT_MINIMUM = 0;
    public static final double SOFT_MAXIMUM = 0.4;
    
    public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
    public static final int MOTOR_ID = 24;
    public static final double GEAR_RATIO = 9;
    public static final boolean IS_INVERTED = false;
    public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
    public static final double MOTOR_FEED_FORWARD = 0.2;
    public static final PIDFGains ANGLE_PID = new PIDFGains(1,0,0);
    public static final boolean IS_BRUSHLESS = true;
    }
    public static class DropperMotor
    {
        public static final double POWER_TO_REACH = 0.8; 
        public static final boolean IS_INVERTED = true;
        public static final int ID = 20;
        public static final double MAX_DROPPER_POWER = 0.8;
        public static final boolean BRUSHLESS = true;
    }

    public static double HIGH_BALL_DROP_TIME = 1;
    public static double LOW_BALL_DROP_TIME = 1;

    
    public static final double X_ON_ROBOT = 0.24;
    public static final double Y_ON_ROBOT = 0.13;
    public static final double Z_OFFSET = 0.87;

}
