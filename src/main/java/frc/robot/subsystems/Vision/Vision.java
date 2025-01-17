// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.List;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;



public class Vision extends SubsystemBase {

  private  final  AprilTagFieldLayout fieldLayout;
  @AutoLog
  public static class VisionInputs{
    public boolean hasTarget = false;
    public Pose3d estimatedPose = new Pose3d();
    public double timeStamp;
    public double latency;
  }

  private final PhotonCamera[] cameras;
  private final PhotonPoseEstimator[] poseEstimators;
  private final VisionInputsAutoLogged[] inputs;

  private final VisionConsumer poseConsumer;
  /** Creates a new Vision. */
  public Vision(VisionConsumer poseConsumer) {
    fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

    int numOfCameras = CameraConstants.values().length;

    cameras = new PhotonCamera[numOfCameras];
    poseEstimators = new PhotonPoseEstimator[numOfCameras];
    inputs = new VisionInputsAutoLogged[numOfCameras];

    CameraConstants[] constants = CameraConstants.values();

    for(int i = 0; i < numOfCameras; i++){
      cameras[i] = new PhotonCamera(constants[i].cameraName);
      inputs[i] = new VisionInputsAutoLogged();

      PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        fieldLayout,
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        constants[i].robotToCamera);

      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      poseEstimators[i] = poseEstimator;                                                                                                          
    }

    this.poseConsumer = poseConsumer;
  }

  @Override
  public void periodic() {
    for(int i = 0; i < cameras.length; i++){
      PhotonCamera camera = cameras[i];
      PhotonPoseEstimator poseEstimator = poseEstimators[i];

      List<PhotonPipelineResult> results = camera.getAllUnreadResults();

      EstimatedRobotPose lastPose = null;

      for(PhotonPipelineResult result : results) {
        if(!result.hasTargets()) continue;

        double poseAmbiguity;
        Pose3d robotPose;
        double timeStamp = result.getTimestampSeconds();

        var estimatedPose = poseEstimator.update(result);

        if(estimatedPose.isEmpty()) continue;

        robotPose = estimatedPose.get().estimatedPose;

        boolean rejectPoseX =
                robotPose.getX() < 0
              ||robotPose.getX() > fieldLayout.getFieldLength();
        boolean rejectPoseY =
                robotPose.getY() < 0
              ||robotPose.getY() > fieldLayout.getFieldWidth();
        boolean rejectPoseZ =
                Math.abs(robotPose.getZ()) > VisionConstants.maxHightDeveation;
        boolean rejectPose = rejectPoseX || rejectPoseY || rejectPoseZ;
        if (rejectPose) continue;


        if(result.multitagResult.isPresent()){
          poseAmbiguity = result.multitagResult.get().estimatedPose.ambiguity;
          if (poseAmbiguity > VisionConstants.maxMultiAmbiguity) continue;

        }else{
          poseAmbiguity = result.getTargets().get(0).poseAmbiguity;
          if (poseAmbiguity > VisionConstants.maxSingleAmbiguity) continue;
        }

        lastPose = estimatedPose.get();

        poseConsumer.accept(robotPose.toPose2d(), timeStamp, VecBuilder.fill(poseAmbiguity, poseAmbiguity, poseAmbiguity));
      }

      
      inputs[i].hasTarget = results.get(results.size() - 1).hasTargets();

      if(lastPose != null){
        inputs[i].estimatedPose = lastPose.estimatedPose;
        inputs[i].timeStamp = lastPose.timestampSeconds;
        inputs[i].latency = RobotController.getMeasureTime().in(Units.Seconds) - inputs[i].timeStamp;
      }else{
        inputs[i].estimatedPose = new Pose3d();
        inputs[i].timeStamp = -1;
        inputs[i].timeStamp = -1;

      }

      Logger.processInputs(VisionConstants.CameraConstants.values()[i].cameraName, inputs[i]);

    }
  
  }

  @FunctionalInterface
  public interface VisionConsumer{
    void accept(Pose2d pose, double timeStamp, Matrix<N3, N1> stdDevs);
  }
}
