package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Pose3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
    VisionFrame[] emptyFrame =
            {new VisionFrame(false, -1, -1, new Pose3d(), 1,
                    EstimationType.SINGLE_TARGET, -1, new Double[]{-1.0})};

    void update(VisionInputsAutoLogged inputs);


     record VisionFrame(
            boolean hasTarget,
            double timeStamp,
            double latency,
            Pose3d robotPose,
            double poseAmbiguity,
            EstimationType estimationType,
            double averageTargetDistance,
            Double[] distanceToTags
    ){}

    enum EstimationType{
        SINGLE_TARGET(VisionConstants.maxSingleAmbiguity),
        MULTIPLE_TARGETS(VisionConstants.maxMultiAmbiguity),
        ;

        EstimationType(double maxAmbiguity) {
            this.maxAmbiguity = maxAmbiguity;
        }

        private final double maxAmbiguity;

        public double getMaxAmbiguity() {
            return maxAmbiguity;
        }
    }

    @AutoLog
    class VisionInputs{
        boolean isConnected;
        VisionFrame[] visionFrames = new VisionFrame[0];
    }

}
