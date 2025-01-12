// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.util.List;
import java.util.Optional;
import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Vision.VisionConstants.CameraConstants;



public class Vision extends SubsystemBase {

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

  private final Consumer<Pair<Pose2d, Double>> poseConsumer;
  /** Creates a new Vision. */
  public Vision(Consumer<Pair<Pose2d, Double>> poseConsumer) {
    AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025Reefscape.loadAprilTagLayoutField();

    int numOfCameras = CameraConstants.values().length;

    cameras = new PhotonCamera[numOfCameras];
    poseEstimators = new PhotonPoseEstimator[numOfCameras];
    inputs = new VisionInputsAutoLogged[numOfCameras];

    CameraConstants[] constants = CameraConstants.values();

    for(int i = 0; i < numOfCameras; i++){
      cameras[i] = new PhotonCamera(constants[i].cameraName);
      inputs[i] = new VisionInputsAutoLogged();

      PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        aprilTagFieldLayout,
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

      for(PhotonPipelineResult result : results){
        if(!result.hasTargets()) continue;

        Optional<EstimatedRobotPose> optionalPose = poseEstimator.update(result);

        if(optionalPose.isEmpty()) continue;

        EstimatedRobotPose pose = optionalPose.get();

        lastPose = pose;

        poseConsumer.accept(new Pair<Pose2d,Double>(pose.estimatedPose.toPose2d(), pose.timestampSeconds));
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
}
