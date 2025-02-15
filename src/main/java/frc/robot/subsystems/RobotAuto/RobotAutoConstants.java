package frc.robot.subsystems.RobotAuto;

import frc.util.PIDFGains;

public class RobotAutoConstants {

    public static class HomingConstants{
        public static double MAX_HOME_SPEED_METERS_PER_SECOND = 1;
        public static double MAX_HOME_SPEED_RADIANS_PER_SECOND = 2.5;

        public static double XY_DEADBAND = 0.2;
        public static double THETA_DEADBAND = 0.5;

        public static double XY_TOLERANCE = 0.0105;
        public static double THETA_TOLERANCE = 0.01;

        public static PIDFGains XY_PID = new PIDFGains(8, 7, 0);
        public static PIDFGains THETA_PID = new PIDFGains(12, 6, 0);
    }

    public enum BallDropTime{
        BEFORE,
        PARALLEL,
        AFTER,
        NEVER
    }
}
