// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.photonvision.EstimatedRobotPose;
import frc.robot.photonvision.PhotonPoseEstimator;

/**
 * A subsystem that controls the PhotonVision tracking on the robot.
 */
public class PhotonVision extends SubsystemBase {
  private PhotonCamera frontLeftCamera;
  private PhotonCamera frontRightCamera;
  private PhotonCamera rearCamera;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator rearPoseEstimator;
  private PhotonPoseEstimator frontLeftPoseEstimator;
  private PhotonPoseEstimator frontRightPoseEstimator;
  private List<PhotonPoseEstimator> estimators = new ArrayList<>();
  private int currentCamera = -1;

  /** Constructs a new Photonvision. */
  public PhotonVision() {
    this.rearCamera = new PhotonCamera("photonvision_rear");
    this.frontLeftCamera = new PhotonCamera("photonvision_front_left");
    this.frontRightCamera = new PhotonCamera("photonvision_front_right");

    try {
      this.aprilTagFieldLayout =
          new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/AprilTagLayout/2023-chargedup.json");
    } catch (IOException io) {
    }

    this.rearPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, rearCamera,
            VisionConstants.ROBOT_TO_REAR_CAMERA, 0);

    this.frontLeftPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, frontLeftCamera,
            VisionConstants.ROBOT_TO_FRONT_LEFT_CAMERA, 1);

    this.frontRightPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, frontRightCamera,
            VisionConstants.ROBOT_TO_FRONT_RIGHT_CAMERA, 2);

    estimators.add(rearPoseEstimator);
    estimators.add(frontLeftPoseEstimator);
    estimators.add(frontRightPoseEstimator);
  }

  /**
   * Returns the latest camera result from the front left camera.
   */
  public PhotonPipelineResult getFrontLeftLatestResult() {
    return frontLeftCamera.getLatestResult();
  }

  /**
   * Returns the latest camera result from the front right camera.
   */
  public PhotonPipelineResult getFrontRightLatestResult() {
    return frontRightCamera.getLatestResult();
  }

  /**
   * Returns the latest camera result from the rear camera.
   */
  public PhotonPipelineResult getRearLatestResult() {
    return rearCamera.getLatestResult();
  }

  /**
   * Returns a List of the visible AprilTags from the front camera.
   */
  public List<PhotonTrackedTarget> getFrontTargets() {
    List<PhotonTrackedTarget> targets = this.getFrontRightLatestResult().getTargets();
    targets.addAll(this.getFrontLeftLatestResult().getTargets());
    return targets;
  }

  /**
   * Returns a List of the visible AprilTags from the rear camera.
   */
  public List<PhotonTrackedTarget> getRearTargets() {
    return this.getRearLatestResult().getTargets();
  }

  /**
   * Returns a List of the IDs of the visible AprilTags.
   */
  public List<Integer> getFiducialIDs() {
    List<Integer> ids = new ArrayList<>();
    for (PhotonTrackedTarget target : this.getFrontTargets()) {
      ids.add(target.getFiducialId());
    }
    for (PhotonTrackedTarget target : this.getRearTargets()) {
      ids.add(target.getFiducialId());
    }
    return ids;
  }

  /**
   * Returns a list of estimated robot poses from the three {@link PhotonPoseEstimator}.
   */
  public List<EstimatedRobotPose> getGlobalPoses() {
    List<EstimatedRobotPose> poses = new ArrayList<>();

    for (PhotonPoseEstimator estimator : estimators) {
      Optional<EstimatedRobotPose> result = estimator.update();
      if (result.isPresent()) {
        poses.add(result.get());
      }
    }
    return poses;
  }

  /**
   * Estimates the global field pose based on a selected {@link PhotonPoseEstimator}.
   */
  public EstimatedRobotPose getCurrentPose() {
    double bestConfidence = 0;
    EstimatedRobotPose bestPose = null;
    int bestId = -1;
    for (PhotonPoseEstimator estimator : estimators) {
      Optional<EstimatedRobotPose> result = estimator.update();
      if (result.isPresent()) {
        EstimatedRobotPose pose = result.get();
        if (estimator.id == currentCamera && pose.confidence > VisionConstants.MAINTAIN_CAMERA_CONFIDENCE_THRESHOLD) {
          return pose;
        }
        if (pose.confidence > bestConfidence) {
          bestConfidence = pose.confidence;
          bestPose = pose;
          bestId = estimator.id;
        }
      }
    }
    currentCamera = bestId;
    SmartDashboard.putString("Vision Pose", bestPose.toString());
    return bestPose;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Current Camera Used:", currentCamera);
  }
}
