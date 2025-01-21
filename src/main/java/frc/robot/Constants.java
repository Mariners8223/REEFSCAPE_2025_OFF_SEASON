// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/**
 * Add your docs here.
 */
public class Constants {
    public enum RobotType {
        COMPETITION,
        DEVELOPMENT,
        REPLAY
    }
    public enum ReefLocation {
        REEF_1(3.173, 3.860, 0),
        REEF_2(3.173, 4.190, 0),
        REEF_3(3.686, 5.086, -60),
        REEF_4(3.975, 5.246, -60),
        REEF_5(5.053, 5.335, -120),
        REEF_6(5.29, 5.08, -120),
        REEF_7(5.803, 4.19, -180),
        REEF_8(5.803, 3.86, -180),
        REEF_9(5.289, 2.967, 120),
        REEF_10(5.005, 2.807, 120),
        REEF_11(3.973, 2.804, 60),
        REEF_12(3.686, 2.97, 60);

        private Pose2d pose;

        public Pose2d getPose() {
            return pose;
        }

        public boolean isBallInUpPosition(){
            return switch (this){
                case REEF_1, REEF_2, REEF_5, REEF_6, REEF_9, REEF_10 -> true;
                case REEF_3, REEF_4, REEF_7, REEF_8, REEF_11, REEF_12 -> false;
            };
        }

        public boolean isBallDropInSamePose(){
            return (this.ordinal() + 1)%2 == 0;
        }

        public static void checkAlliance(boolean isBlue) {
            if (isBlue) return;

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

    public static final RobotType ROBOT_TYPE = RobotType.DEVELOPMENT; //the type of robot the code is running on
}
}