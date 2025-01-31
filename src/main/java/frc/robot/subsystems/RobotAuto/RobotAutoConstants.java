package frc.robot.subsystems.RobotAuto;

import frc.util.PIDFGains;

public class RobotAutoConstants {

    public static class HomingConstants{
        public static double MAX_HOME_SPEED_METERS_PER_SECOND = 1;
        public static double MAX_HOME_SPEED_RADIANS_PER_SECOND = 2.5;

        public static double XY_TOLERANCE = 0.005;
        public static double THETA_TOLERANCE = 0.005;

        public static PIDFGains XY_PID = new PIDFGains(7, 10, 0, 0);
        public static PIDFGains THETA_PID = new PIDFGains(12, 0, 0, 0);
    }

    public enum BallDropTime{
        BEFORE,
        PARALLEL,
        AFTER
    }
}
