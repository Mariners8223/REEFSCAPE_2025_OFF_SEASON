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

    public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static final double maxHeightDeviation = 0.1;
    public static final double maxMultiAmbiguity = 0.3;
    public static final double maxSingleAmbiguity = 0.1;

    public enum CameraConstants{
        END_EFFECTOR_CAMERA("EndEffectorCamera",
            new Transform3d(
                0.205, 0.081, 0.32,
                new Rotation3d(0, Units.degreesToRadians(-1), Units.degreesToRadians(-15))),
                0.2, 0.16), 

       FUNNEL_CAMERA("FunnelCamera",
           new Transform3d(
               -0.34, 0.178, 0.557,
               new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(178))),
               0.2, 0.16);

        public final String cameraName;
        public final double XYstdFactor;
        public final double thetaStdFactor;

        public final Transform3d robotToCamera;

        CameraConstants(String name, Transform3d robotToCamera, double xySTD, double thetaSTD){
            this.cameraName = name;
            this.robotToCamera = robotToCamera;
            this.XYstdFactor = xySTD;
            this.thetaStdFactor = thetaSTD;
        }

        @Override
        public String toString(){
            return cameraName;
        }
    }

}