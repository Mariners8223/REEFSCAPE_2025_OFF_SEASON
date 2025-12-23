package main.java.frc.robot.subsystems.shooter;
import frc.util.PIDFGains;
import frc.util.MarinersController.MarinersController.ControllerLocation;

public class ShooterConstants {
    double FRONTPOWER;
    double BACKPOWER 5 * -1;
    double SHOOTINGANGLE;
    double TIMEFORSTOP = 150;//for 3 seconds of spinning after isGpIn is false
    double POWERSTOP = 0;

    public static final double MAX_MOTOR_POWER = 0.8;
    public static final boolean BEAM_BREAK_INVERTED = false;

    public static final int BEAM_BREAK_PORT = 2; //TODO: find right value
    
    public static class FrontMotor
    {
        public static final int MOTOR_ID = 23; //TODO: fix vlaues
        public static final boolean IS_INVERTED = true; //TODO: fix vlaues
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR; //TODO: fix values
        public static final PIDFGains PID_GAINS = new PIDFGains(
            200,
            2,
            0,
            0,
            0.0,
            0.05);
    }
    
    public static class BackMotor
    {
        public static final int MOTOR_ID = 24; //TODO: fix values
        public static final boolean BACK_INVERTED = true; //TODO: fix values
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR; //TODO: fix values
        public static final PIDFGains PID_GAINS = new PIDFGains(
            200,
            2,
            0,
            0,
            0.0,
            0.05);
    }

    public static class AngleMotor  
    {
        public static final int ANGLE_ID = 24;
        public static final boolean ANGLE_INVERTED = true; //TODO: find right value
        public static final ControllerLocation CONTROLLER_LOCATION = ControllerLocation.MOTOR; //TODO: fix values
        public static final double GEAR_RATIO = 45;
        public static final PIDFGains PID_GAINS = new PIDFGains(
            200,
            2,
            0,
            0,
            0.0,
            0.05);
    }

}
