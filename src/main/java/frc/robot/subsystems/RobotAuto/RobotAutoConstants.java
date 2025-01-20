package frc.robot.subsystems.RobotAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public class RobotAutoConstants {

    public static class HomingConstants{
        public static double MAX_HOME_SPEED_METERS_PER_SECOND = 0.5;
        public static double MAX_HOME_SPEED_RADIANS_PER_SECOND = 0.5;

        public static double XY_TOLERANCE = 0.1;
        public static double THETA_TOLERANCE = 0.1;
    }
}
