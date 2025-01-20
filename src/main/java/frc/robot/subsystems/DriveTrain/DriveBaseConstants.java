package frc.robot.subsystems.DriveTrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.DriveTrain.SwerveModules.CompBotConstants;
import frc.robot.subsystems.DriveTrain.SwerveModules.DevBotConstants;
import frc.util.PIDFGains;

public class DriveBaseConstants {

    public static final double DISTANCE_BETWEEN_WHEELS = 0.58; // the distance between each wheel in meters
    public static final Translation2d[] MODULE_TRANSLATIONS = new Translation2d[]{
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, DISTANCE_BETWEEN_WHEELS / 2),
            new Translation2d(-DISTANCE_BETWEEN_WHEELS / 2, -DISTANCE_BETWEEN_WHEELS / 2)};


    public static final class PathPlanner {
        public static final ModuleConfig MODULE_CONFIG = Constants.ROBOT_TYPE == RobotType.DEVELOPMENT ?
                DevBotConstants.MODULE_CONFIG :
                CompBotConstants.MODULE_CONFIG;

        public static final RobotConfig ROBOT_CONFIG = new RobotConfig(
                20,
                6.883,
                MODULE_CONFIG,
                MODULE_TRANSLATIONS);

        public static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(
                MODULE_CONFIG.maxDriveVelocityMPS,
                10, //TODO find a good value for this
                10,
                5); //the constraints for pathPlanner

        public static final PIDFGains THETA_PID = new PIDFGains(7, 0, 0); //the pid gains for the PID Controller of the robot angle, units are radians per second
        public static final PIDFGains XY_PID = new PIDFGains(5.5, 0.055, 0.05); //the pid gains for the pid controller of the robot's velocity, units are meters per second
    }

    public static final int PIGEON_ID = 2;
}
