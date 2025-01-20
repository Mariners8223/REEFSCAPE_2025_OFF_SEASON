// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Add your docs here.
 */
public class Constants {
    public enum RobotType {
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }

    public static final RobotType ROBOT_TYPE = RobotType.DEVELOPMENT; //the type of robot the code is running on

    public enum ReefLocation {
        //HEY
        REEF_1(3.14, 3.85, 0),
        REEF_2(3.14, 4.2, 0),
        REEF_3(3.645, 5.07, -60),
        REEF_4(3.96, 3.1, -60),
        REEF_5(5.02, 5.29, -120),
        REEF_6(5.34, 5.146, -120),
        REEF_7(5.85, 4.21, -180),
        REEF_8(5.85, 3.88, -180),
        REEF_9(5.51, 2.93, 120),
        REEF_10(5.234, 2.8, 120),
        REEF_11(4.06, 2.76, 60),
        REEF_12(3.84, 2.91, 60);

        private Pose2d pose;

        public Pose2d getPose() {
            return pose;
        }

        public static void checkAlliance(boolean isBlue){
            if(!isBlue) return;

            AprilTagFields fields = AprilTagFields.k2025Reefscape;

            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(fields);

            for (ReefLocation r : ReefLocation.values()) {
                r.pose = new Pose2d(
                        layout.getFieldLength() - r.pose.getX(),
                        layout.getFieldWidth() - r.pose.getY(),
                        Rotation2d.fromDegrees(r.pose.getRotation().getDegrees() - 180));
            }
        }

        ReefLocation(double x, double y, double deg) {
            pose = new Pose2d(x, y, Rotation2d.fromDegrees(deg));
        }
    }

    public static enum FeederLocation {
        LEFT(0, 0,
                0, 0,
                0, 0,
                0, 0,
                0, 0, 0
        ),

        RIGHT(0, 0,
                0, 0,
                0, 0,
                0, 0,
                0, 0, 0
        );

        //order is:
        //topRight corner
        //topLeft corner
        //bottomRight corner
        //bottomLeft corner
        private final Translation2d[] corners = new Translation2d[4];

        private Pose2d robotPose;

        public Translation2d getCorner(int index){
            return corners[index];
        }

        public Pose2d getRobotPose(){
            return robotPose;
        }

        public static void checkAlliance(boolean isBlue){
            if(!isBlue) return;

            Translation2d[] prevLeftFeeder = LEFT.corners.clone();
            Translation2d[] prevRightFeeder = RIGHT.corners.clone();

            AprilTagFields fields = AprilTagFields.k2025Reefscape;

            AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(fields);

            for(int i = 0; i < 4; i++){
                RIGHT.corners[i] = new Translation2d(layout.getFieldLength() - prevLeftFeeder[i].getX(),
                        prevLeftFeeder[i].getY());

                LEFT.corners[i] = new Translation2d(layout.getFieldLength() - prevRightFeeder[i].getX(),
                        prevRightFeeder[i].getY());
            }


            Pose2d prevLeftPose = LEFT.robotPose;
            Pose2d prevRightPose = RIGHT.robotPose;

            RIGHT.robotPose = new Pose2d(layout.getFieldLength() - prevLeftPose.getX(), prevLeftPose.getY(),
                    Rotation2d.fromDegrees(prevLeftPose.getRotation().getDegrees() - 180));

            LEFT.robotPose = new Pose2d(layout.getFieldLength() - prevRightPose.getX(), prevRightPose.getY(),
                    Rotation2d.fromDegrees(prevRightPose.getRotation().getDegrees() - 180));
        }

        FeederLocation(double topRightX, double topRightY, double topLeftX, double topLeftY,
                       double bottomRightX,double bottomRightY, double bottomLeftX, double bottomLeftY,
                       double robotPoseX, double robotPoseY, double robotPoseDeg){

            corners[0] = new Translation2d(topRightX, topRightY);
            corners[1] = new Translation2d(topLeftX, topLeftY);
            corners[2] = new Translation2d(bottomRightX, bottomRightY);
            corners[3] = new Translation2d(bottomLeftX, bottomLeftY);

            robotPose = new Pose2d(robotPoseX, robotPoseY, Rotation2d.fromDegrees(robotPoseDeg));
        }
    }
}