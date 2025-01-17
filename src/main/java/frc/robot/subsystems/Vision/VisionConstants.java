package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import org.photonvision.PhotonPoseEstimator;

public class VisionConstants {

    public static final PhotonPoseEstimator.PoseStrategy MAIN_STRATEGY =
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final PhotonPoseEstimator.PoseStrategy FALLBACK_STRATEGY =
            PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY;

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    //TODO test to find better numbers
    public static final double maxHeightDeviation = 0.05;
    public static final double maxMultiAmbiguity = 0.5;
    public static final double maxSingleAmbiguity = 0.3;


    public enum CameraConstants{
        FRONT_LEFT("front left",
            new Transform3d(
                0.11, -0.31, 0.167,
                new Rotation3d(0, Units.degreesToRadians(15), Units.degreesToRadians(-90)))),

        BACK_MIDDLE("back middle",
            new Transform3d(
                0, 0, 0,
                new Rotation3d(0, 0, 0)));

        public final String cameraName;

        public final Transform3d robotToCamera;

        CameraConstants(String name, Transform3d robotToCamera){
            this.cameraName = name;
            this.robotToCamera = robotToCamera;
        }

        @Override
        public String toString(){
            return cameraName;
        }
    }

}
