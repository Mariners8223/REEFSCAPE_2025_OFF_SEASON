package frc.robot.subsystems.RobotAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;

import java.util.ArrayList;
import java.util.List;

public class RobotAutoConstants {

    public static ArrayList<Pose2d> reefPoses = new ArrayList<>();

    public static enum FeederLocations{
        TOP,
        BOTTOM;

        public final Translation2d topRight;
        public final Translation2d topLeft;
        public final Translation2d bottomRight;
        public final Translation2d bottomLeft;

        public final Pose2d robotPose;

        FeederLocations(Translation2d topRight, Translation2d topLeft, Translation2d bottomLeft, Translation2d bottomRight, Pose2d robotPose){
            if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
                this.topRight = new Translation2d(16 - topRight.getX(), topRight.getY());
                this.topLeft = new Translation2d(16 - topLeft.getX(), topLeft.getY());
                this.bottomLeft = new Translation2d(16 - bottomLeft.getX(), bottomLeft.getY());
                this.bottomRight = new Translation2d(16 - bottomRight.getX(), bottomRight.getY());
                this.robotPose = new Pose2d(16 - robotPose.getX(), robotPose.getY(), Rotation2d.fromDegrees(180 - robotPose.getRotation().getDegrees()));
            }
            else{
                this.topRight = topRight;
                this.topLeft = topLeft;
                this.bottomRight = bottomRight;
                this.bottomLeft = bottomLeft;
                this.robotPose = robotPose;
            }
        }
    }
}
