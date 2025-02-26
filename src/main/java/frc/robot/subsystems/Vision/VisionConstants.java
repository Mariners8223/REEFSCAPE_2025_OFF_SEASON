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

    public static final double maxHeightDeviation = 0.1;
    public static final double maxMultiAmbiguity = 0.3;
    public static final double maxSingleAmbiguity = 0.1;

	public static final int X_PIXELS = 600;
	public static final int MIDLINE_X = 300;
    public static final int Y_PIXELS = 500;

    public static double XYstdFactor = 0.06;
    public static double thetaStdFactor = 0.08;


    public enum CameraConstants{
        END_EFFECTOR_CAMERA("EndEffectorCamera",
            new Transform3d(
                0.2, 0.09, 0.297,
                // 0, 0, 0,
                new Rotation3d(0, Units.degreesToRadians(-1), Units.degreesToRadians(-13)))), 
                // new Rotation3d(0,0,0))),

       FUNNEL_CAMERA("FunnelCamera",
           new Transform3d(
               -0.375
               , 0.165, 0.56,
            //    0, 0, 0,
               new Rotation3d(0, Units.degreesToRadians(-12), Units.degreesToRadians(178))));
            //    new Rotation3d(0,0,0)));

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