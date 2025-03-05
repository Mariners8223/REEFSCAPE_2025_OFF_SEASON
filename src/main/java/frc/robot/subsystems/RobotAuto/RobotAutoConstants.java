package frc.robot.subsystems.RobotAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;


public class RobotAutoConstants {

        public static final double LOWER_SPEED_LIMIT_XY = 1.5;
        public static final double UPPER_SPEED_LIMIT_XY = 3;

        public static double LOWER_SPEED_LIMIT_THETA = 1;
        public static final double UPPER_SPEED_LIMIT_THETA = 4;

        public static final double XY_DEADBAND = 0;
        public static final double THETA_DEADBAND = 0.4;

        public static final double XY_TOLERANCE = 0.0105;
        public static final double THETA_TOLERANCE = Units.degreesToRadians(2);

        public static final double FAR_FROM_TARGET_DISTANCE = 1;

        public static final double THETA_IZONE = Units.degreesToRadians(10);

        public static final PIDController TRANSLATION_PID = new PIDController(30, 800, 6);
        public static final PIDController THETA_PID = new PIDController(14, 3, 1);

        static{
            TRANSLATION_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            THETA_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

            TRANSLATION_PID.setIZone(0.1);
            THETA_PID.setIZone(THETA_IZONE);

            TRANSLATION_PID.setTolerance(XY_TOLERANCE);
            THETA_PID.setTolerance(THETA_TOLERANCE);

            TRANSLATION_PID.setSetpoint(0);
        }
}
