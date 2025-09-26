package frc.robot.subsystems.Elevator;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

        private double height;

  
        private ElevatorLevel (double height) {
            this.height = height;
        }


        public double getHeight() {
            return height;
        }
    }



        
    public static class LeadMotor{
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR;
        public static final int MOTOR_ID = 17;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;
        
        public static final boolean IS_INVERTED = false;

        public static final TrapezoidProfile TRAPEZOID_PROFILE = new TrapezoidProfile(
            new Constraints(5, 12)
        );
    }
    

    public static class FollowMotor{
        public static final  ControllerLocation CONTROLLER_LOCATION = ControllerLocation.RIO;
        public static final int MOTOR_ID = 16;
        public static final boolean IS_BRUSHLESS = true;
        public static final MotorType MOTOR_TYPE = MotorType.SPARK_FLEX;

        public static final boolean IS_INVERTED = true;

    }

    public static final double SOFT_MINIMUM = ElevatorLevel.BottomHeight.getHeight();
    public static final double SOFT_MAXIMUM = ElevatorLevel.L4.getHeight();
    
    public static final double GEAR_RAIO = 5;
    public static final double PULLEY_RADIUS = 0.024;
    public static final double PULLEY_EXTENSION_RATIO = PULLEY_RADIUS * 2 * Math.PI * 2;


    
}
