package frc.robot.subsystems.shooterGali;

import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

public class ShooterConstants 
{
    //TODO: Find all constants!!!
    public static class FrontMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 17;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = false;
    } 

    public static class BackMotor
    {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 16;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        public static final boolean IS_INVERTED = true;
    }

    public static class AngleMotor
    {
        public static final double ANGLE_TOLERANCE = 0.03;
        public static final double ANGLE_TO_RESET = 0;
        public static final double ANGLE_TO_REACH = 45;
    }
    
    public static final double POWER_TO_REACH = 0.8; //what power to use for the shoot, need to check
    public static final double GRAVITY_PHISICS = 9.8066;

}
