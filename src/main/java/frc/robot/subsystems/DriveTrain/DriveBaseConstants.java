package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;
import frc.util.PIDFGains;

public class DriveBaseConstants {

    public static final double DISTANCE_BETWEEN_WHEELS = 0.58; // the distance between each wheel in meters
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2)};

    public static final double THETA_KS = 0.20501;
    public static final double THETA_KV = 0.83139;
    public static final double THETA_KA = 0.09286;

    public static final class PathPlanner {
        public static final ModuleConfig MODULE_CONFIG = DevBotConstants.MODULE_CONFIG;

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                53.3,
                5.502692,
                MODULE_CONFIG,
                MODULE_TRANSLATIONS);

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                3.5,
                6, //TODO find a good value for this
                5,
                25); //the constraints for pathPlanner

        public static final PIDFGains THETA_PID = new PIDFGains(1.7, 0.01, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
        public static final PIDFGains XY_PID = new PIDFGains(2, 0.2, 0.02);//the pid gains for the pid controller of the robot's velocity, units are meters per second
    }

    public static final int PIGEON_ID = 2;
}
