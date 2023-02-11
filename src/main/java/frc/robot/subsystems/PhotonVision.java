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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EstimatedRobotPose;
import frc.robot.PhotonPoseEstimator;
import frc.robot.Constants.VisionConstants;

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
            VisionConstants.REAR_CAMERA_TO_ROBOT);

    this.frontLeftPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, frontLeftCamera,
            VisionConstants.FRONT_LEFT_CAMERA_TO_ROBOT);

    this.frontRightPoseEstimator =
        new PhotonPoseEstimator(aprilTagFieldLayout, frontRightCamera,
            VisionConstants.FRONT_RIGHT_CAMERA_TO_ROBOT);

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
   * Estimates the global field pose based on previous pose and an {@link RobotPoseEstimator}.
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

  public EstimatedRobotPose getAveragedGlobalPose() {
    List<EstimatedRobotPose> visionEstimatedPoses = this.getGlobalPoses();
    Pose2d estimatedVisionPose = new Pose2d();
    Rotation2d averageEstimatedRotation = new Rotation2d();
    double averageEstimatedX = 0;
    double averageEstimatedY = 0;
    double averageTimestamp = 0;
    double sumConfidence = 0;

    for (EstimatedRobotPose pose : visionEstimatedPoses) {
      if (pose.confidence > 0) {
        averageEstimatedRotation = averageEstimatedRotation.plus(pose.estimatedPose.getRotation());
        averageEstimatedX += pose.estimatedPose.getX() * pose.confidence;
        averageEstimatedY += pose.estimatedPose.getY() * pose.confidence;
        averageTimestamp += pose.timestampSeconds * pose.confidence;
        sumConfidence += pose.confidence;
      }
    }

    averageEstimatedRotation = averageEstimatedRotation.div(visionEstimatedPoses.size() * sumConfidence);
    averageEstimatedX /= visionEstimatedPoses.size() * sumConfidence;
    averageEstimatedY /= visionEstimatedPoses.size() * sumConfidence;
    averageTimestamp /= visionEstimatedPoses.size() * sumConfidence;
    estimatedVisionPose = new Pose2d(averageEstimatedX, averageEstimatedY, averageEstimatedRotation);
    return new EstimatedRobotPose(estimatedVisionPose, averageTimestamp, sumConfidence);
  }


}
