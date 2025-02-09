package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;

import org.photonvision.PhotonPoseEstimator;

public class VisionConstants {

    public static final PhotonPoseEstimator.PoseStrategy MAIN_STRATEGY =
            PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

    public static final PhotonPoseEstimator.PoseStrategy FALLBACK_STRATEGY =
            PhotonPoseEstimator.PoseStrategy.CLOSEST_TO_CAMERA_HEIGHT;

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    public static final double maxHeightDeviation = 0.1;
    public static final double maxMultiAmbiguity = 0.3;
    public static final double maxSingleAmbiguity = 0.1;

    public static double XYstdFactor = 0.02;
    public static double thetaStdFactor = 0.02;



    public enum CameraConstants{
        END_EFFECTOR("EndEffector",
            new Transform3d(
                0.2, 0.1, 0.297,
                //0, 0, 0,
                new Rotation3d(0, Units.degreesToRadians(-1), Units.degreesToRadians(-17)))), 

       FUNNEL("Funnel",
           new Transform3d(
               -0.3, -0.18, 0.56,
               new Rotation3d(0, Units.degreesToRadians(-14), Units.degreesToRadians(180))));

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