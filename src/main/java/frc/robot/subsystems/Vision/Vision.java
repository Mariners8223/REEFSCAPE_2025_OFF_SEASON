// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.ArrayList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;


public class Vision extends SubsystemBase {

    private final VisionCamera[] cameras;

    private final VisionConsumer poseConsumer;

    /**
     * Creates a new Vision.
     */
    public Vision(VisionConsumer poseConsumer) {

        int numOfCameras = CameraConstants.values().length;

        cameras = new VisionCamera[numOfCameras];

        CameraConstants[] constants = CameraConstants.values();

        for (int i = 0; i < numOfCameras; i++) {
            if(Constants.ROBOT_TYPE != RobotType.REPLAY) cameras[i] = new VisionCamera(constants[i], VisionConstants.FIELD_LAYOUT);
            else cameras[i] = new VisionCamera(constants[i].cameraName);
        }

        this.poseConsumer = poseConsumer;
    }

    @Override
    public void periodic() {
        for (VisionCamera camera : cameras) {
            camera.update();
            camera.log();

            // For logging purposes
            ArrayList<Pose3d> acceptedPoses = new ArrayList<>();
            ArrayList<Pose3d> rejectedPoses = new ArrayList<>();

            for (VisionIO.VisionFrame frame : camera.inputs.visionFrames) {
                if (!frame.hasTarget()) continue;

                if (!checkPoseLocation(frame.robotPose())) {
                    rejectedPoses.add(frame.robotPose());
                    continue;
                }

                if (!checkPoseAmbiguity(frame.poseAmbiguity(), frame.estimationType())) {
                    rejectedPoses.add(frame.robotPose());
                    continue;
                }

                var stdDevs = getStdDevs(frame.averageTargetDistance(), frame.tagCount(), frame.estimationType());

                poseConsumer.accept(frame.robotPose().toPose2d(), frame.timeStamp(), stdDevs);

                acceptedPoses.add(frame.robotPose());
            }

            Logger.recordOutput("Vision/" + camera.cameraName + "/Accepted Poses", acceptedPoses.toArray(new Pose3d[0]));
            Logger.recordOutput("Vision/" + camera.cameraName + "/Rejected Poses", rejectedPoses.toArray(new Pose3d[0]));
        }

    }

    /**
     * If the pose is in a valid location.
     * The pose is valid if it is within the field
     * and the height of the robot is in tolerance
     *
     * @param pose the calculated pose
     * @return whether the pose is in a valid location
     */
    private boolean checkPoseLocation(Pose3d pose) {
        double poseX = pose.getX();
        double poseY = pose.getY();
        double poseZ = pose.getZ();

        if (poseX >= VisionConstants.FIELD_LAYOUT.getFieldLength() || poseX < 0) return false;
        if (poseY >= VisionConstants.FIELD_LAYOUT.getFieldWidth() || poseY < 0) return false;
        return !(Math.abs(poseZ) > VisionConstants.maxHeightDeviation);
    }

    /**
     * Checks if the pose ambiguity is good enough for proceeding
     *
     * @param poseAmbiguity  the ambiguity of the pose (0 is no ambiguity, 1 is no idea)
     * @param estimationType the type of estimation used
     * @return whether the pose ambiguity is within acceptable bounds
     */
    private boolean checkPoseAmbiguity(double poseAmbiguity, VisionIO.EstimationType estimationType) {
        if (estimationType == null) {
            return false;
        }
        return poseAmbiguity <= estimationType.getMaxAmbiguity();
    }

        /**
     * calculates the standard deviations for the pose based on the average distance to the tags
     *
     * @param averageTagDistance the average distance to the tags
     * @param estimationType     the type of estimation used
     * @return the standard deviations for the pose
     */
    private Matrix<N3, N1> getStdDevs(double averageTagDistance, double tagCount, VisionIO.EstimationType estimationType) {
        // Calculate standard deviations
        double stdDevFactor =
                Math.pow(averageTagDistance, 2.0) / tagCount;

        double linearStdDev = VisionConstants.XYstdFactor * stdDevFactor;
        double angularStdDev = VisionConstants.thetaStdFactor * stdDevFactor;
        if (estimationType == VisionIO.EstimationType.MULTIPLE_TARGETS) {
            linearStdDev *= VisionConstants.XYstdFactor;
            angularStdDev *= VisionConstants.thetaStdFactor;
        }

        return VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev);
    }



    @FunctionalInterface
    public interface VisionConsumer {
        void accept(Pose2d pose, double timeStamp, Matrix<N3, N1> stdDevs);
    }


    private static class VisionCamera {
        private final VisionIO camera;
        private final String cameraName;
        private final VisionInputsAutoLogged inputs;

        public VisionCamera(VisionIO camera, String cameraName) {
            this.camera = camera;
            this.cameraName = cameraName;
            this.inputs = new VisionInputsAutoLogged();
        }

        public VisionCamera(CameraConstants constants, AprilTagFieldLayout fieldLayout) {
            this(new VisionIOPhoton(constants, fieldLayout), constants.cameraName);
        }

        public VisionCamera(String cameraName){
            this.camera = new VisionIO() {
                @Override
                public void update(VisionInputsAutoLogged inputs) {
                }
            };

            this.cameraName = cameraName;
            inputs = new VisionInputsAutoLogged();

        }

        public void update() {
            camera.update(inputs);
        }

        public void log() {
            Logger.processInputs(cameraName, inputs);
        }
    }
}
