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

        public static final PIDController X_PID = new PIDController(6, 4, 0.5);
        public static final PIDController Y_PID = new PIDController(6, 4, 0.5);
        public static final PIDController THETA_PID = new PIDController(14, 3, 1);

        static{
            X_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            Y_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);
            THETA_PID.setIntegratorRange(Double.NEGATIVE_INFINITY, Double.POSITIVE_INFINITY);

            X_PID.setIZone(0.1);
            Y_PID.setIZone(0.1);
            THETA_PID.setIZone(THETA_IZONE);

            X_PID.setTolerance(XY_TOLERANCE);
            Y_PID.setTolerance(XY_TOLERANCE);
            THETA_PID.setTolerance(THETA_TOLERANCE);

            THETA_PID.enableContinuousInput(-Math.PI, Math.PI);
        }
}
