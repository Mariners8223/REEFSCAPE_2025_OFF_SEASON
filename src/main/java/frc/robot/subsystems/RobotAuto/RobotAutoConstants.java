package frc.robot.subsystems.RobotAuto;

import edu.wpi.first.math.util.Units;
import frc.util.PIDFGains;

public class RobotAutoConstants {

    public static class HomingConstants{
        public static double MAX_HOME_SPEED_METERS_PER_SECOND = 0.8;
        public static double MAX_HOME_SPEED_RADIANS_PER_SECOND = 2.5;

        public static double LOWER_SPEED_LIMIT_XY = 0.8;
        public static double UPPER_SPEED_LIMIT_XY = 2;

        public static double LOWER_SPEED_LIMIT_THETA = 2.5;
        public static double UPPER_SPEED_LIMIT_THETA = 10;

        public static double XY_DEADBAND = 0.01;
        public static double THETA_DEADBAND = 0.1;

        public static double XY_TOLERANCE = 0.0105;
        public static double THETA_TOLERANCE = Units.degreesToRadians(5);

        // public static PIDFGains XY_PID = new PIDFGains(3, 12, 1, 0, XY_TOLERANCE, 0.03);
        public static PIDFGains XY_PID = new PIDFGains(10, 12, 1, 0, XY_TOLERANCE, 0.03);
        public static PIDFGains THETA_PID = new PIDFGains(14, 20, 0, 0, THETA_TOLERANCE, 0.5);

        public static double FAR_FROM_TARGET_DISTANCE = 1;
    }

    public enum BallDropTime{
        BEFORE,
        PARALLEL,
        AFTER,
        NEVER
    }
}
