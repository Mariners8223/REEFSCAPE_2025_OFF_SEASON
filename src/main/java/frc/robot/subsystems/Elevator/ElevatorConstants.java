package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import frc.util.MarinersController.MarinersController.ControllerLocation;
import frc.util.MarinersController.MarinersSparkBase;
import frc.util.MarinersController.MarinersSparkBase.MotorType;

public class ElevatorConstants {
    public enum ElevatorLevel{
        BottomHeight(0.65),
        L1(10.72),
        L2(20.88),
        L3(1.28),
        L4(1.9);

        private double description;

  
        private void ElevatorConstants (Double description) {
            this.description = description;
        }


        public double getDescription() {
            return description;
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
        public static final  ControllerLocation CONTROLLER_LOCATION = ControllerLocation.RIO;
        public static final int MOTOR_ID = 16;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;

        public static final boolean IS_INVERTED = true;

    }
}
{
public static final double SOFT_MINIMUM = ElevatorLevel.BottomHeight(0.65);
public static final double SOFT_MAXIMUM = 1.9;}