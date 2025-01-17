package frc.robot.subsystems.Vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

public class VisionIOPhoton implements VisionIO {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;


    public VisionIOPhoton(CameraConstants cameraConstants, AprilTagFieldLayout fieldLayout) {
        camera = new PhotonCamera(cameraConstants.cameraName);

        poseEstimator = new PhotonPoseEstimator(fieldLayout, CameraConstants.MAIN_STRATEGY, cameraConstants.robotToCamera);

        poseEstimator.setMultiTagFallbackStrategy(CameraConstants.FALLBACK_STRATEGY);
    }


    @Override
    public void update(VisionInputsAutoLogged inputs) {
        inputs.isConnected = camera.isConnected();

        if(!inputs.isConnected){
            inputs.visionFrames = emptyFrame;
            return;
        }

        var results = camera.getAllUnreadResults();

        inputs.visionFrames = new VisionFrame[results.size()];

        for(int i = 0; i < results.size(); i++){
            var result = results.get(i);

            boolean hasTarget = result.hasTargets();

            if(!hasTarget){
                inputs.visionFrames[i] = emptyFrame[0];
                continue;
            }

            Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update(result);

            if(poseEstimatorResult.isEmpty()){
                inputs.visionFrames[i] = emptyFrame[0];
                continue;
            }

            EstimatedRobotPose robotPose = poseEstimatorResult.get();

            EstimationType estimationType;
            double poseAmbiguity;
            double averageTargetDist = 0;
            if(result.multitagResult.isPresent()){
                estimationType = EstimationType.MULTIPLE_TARGETS;
                poseAmbiguity = result.multitagResult.get().estimatedPose.ambiguity;

                for(PhotonTrackedTarget target : result.getTargets()){
                    averageTargetDist += target.getBestCameraToTarget().getTranslation().getNorm();
                }
            }
            else{
                estimationType = EstimationType.SINGLE_TARGET;
                poseAmbiguity = result.getTargets().get(0).poseAmbiguity;

                averageTargetDist = result.getTargets().get(0).getBestCameraToTarget().getTranslation().getNorm();
            }

            inputs.visionFrames[i] = new VisionFrame(
                    true,
                    result.getTimestampSeconds(),
                    RobotController.getMeasureTime().in(Units.Seconds) - result.getTimestampSeconds(),
                    robotPose.estimatedPose,
                    poseAmbiguity,
                    estimationType,
                    averageTargetDist / result.getTargets().size()
            );
        }

    }
}
