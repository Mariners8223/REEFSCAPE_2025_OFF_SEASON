// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.util.Color;

import org.json.simple.parser.ParseException;

import java.io.IOException;

/**
 * Add your docs here.
 */
public class Constants {
    public enum RobotType {
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }

    public static final RobotType ROBOT_TYPE = RobotType.DEVELOPMENT;

    public enum ReefLocation {
        REEF_1,
        REEF_2,
        REEF_3,
        REEF_4,
        REEF_5,
        REEF_6,
        REEF_7,
        REEF_8,
        REEF_9,
        REEF_10,
        REEF_11,
        REEF_12;

        private Pose2d pose;

        private final PathPlannerPath path;

        public PathPlannerPath getPath() {
            return path;
        }

        public Pose2d getPose() {
            return pose;
        }

        public boolean isBallInUpPosition() {
            return switch (this) {
                case REEF_1, REEF_2, REEF_5, REEF_6, REEF_9, REEF_10 -> true;
                case REEF_3, REEF_4, REEF_7, REEF_8, REEF_11, REEF_12 -> false;
            };
        }

        public boolean isBallDropInSamePose() {
            return (this.ordinal()) % 2 == 0;
        }

        public static void checkAlliance(boolean isBlue) {
            if(isBlue) return;

            for (ReefLocation r : ReefLocation.values()) {

                PathPlannerPath flipped = r.path.flipPath();

                r.pose = getEndPose(flipped);
            }
        }

        ReefLocation() {
            try {
                path = PathPlannerPath.fromPathFile("path to reef " + (this.ordinal() + 1));
            } catch (IOException | ParseException e) {
                throw new RuntimeException(e);
            }

            pose = getEndPose(path);
        }
    }

    public static enum FeederLocation {
        LEFT(1, 2),

        RIGHT(3, 4);

        private PathPlannerPath awayPath;
        private PathPlannerPath closePath;

        public PathPlannerPath getAwayPath(){
            return awayPath;
        }

        public PathPlannerPath getClosePath(){
            return closePath;
        }

        public PathPlannerPath getPath(FeederSide side){
            return side == FeederSide.CLOSE ? closePath : awayPath;
        }

        FeederLocation(int awayNumber, int closeNumber) {
            try {
                awayPath = PathPlannerPath.fromPathFile("path to feeder " + awayNumber);
                closePath = PathPlannerPath.fromPathFile("path to feeder " + closeNumber);
            } catch (FileVersionException | IOException | ParseException e) {
                e.printStackTrace();
            }
        }
    }

    public static enum FeederSide{
        AWAY(new Color(4, 144, 209)),
        CLOSE(Color.kPurple);

        public final Color color;

        FeederSide(Color color){
            this.color = color;
        }

        @Override
        public String toString(){
            switch(this){
                case CLOSE: return "Close";
                default: return "Away";
            }
        }
    }

    private static Pose2d getEndPose(PathPlannerPath path){
        var poses = path.getAllPathPoints();
        var endState = poses.get(poses.size() - 1);

        return new Pose2d(endState.position, endState.rotationTarget.rotation());
    }
}