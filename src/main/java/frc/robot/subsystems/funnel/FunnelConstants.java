
package frc.robot.subsystems.funnel;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import frc.util.MarinersController.MarinersController;
import frc.util.MarinersController.MarinersController.ControllerLocation;

class funnelConstants {
    

    public class FunnelConstants {
        public static final double FUNNEL_ANGLE_HIGH = 0.4;
        public static final double START_ENDGAME = 30;
        public static final double FUNNEL_ANGLE_LOW = 0;
    }
    public static class funnelMotor {
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 17;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.Talon_FX;
        public static final boolean IS_INVERTED = false;
        public static final double ROTATION_TO_ANGLE = 8;
        
    }



}