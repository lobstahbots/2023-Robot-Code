// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * A subsystem that controls the PhotonVision tracking on the robot.
 */
public class PhotonVision extends SubsystemBase {
  private PhotonCamera frontCamera;
  private PhotonCamera rearCamera;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private RobotPoseEstimator robotPoseEstimator;

  private ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<>();

  /** Constructs a new Photonvision. */
  public PhotonVision() {
    this.rearCamera = new PhotonCamera("photonvision_rear");
    this.frontCamera = new PhotonCamera("photonvision_front");

    camList.add(new Pair<PhotonCamera, Transform3d>(frontCamera, VisionConstants.FRONT_CAMERA_TO_ROBOT));
    camList.add(new Pair<PhotonCamera, Transform3d>(rearCamera, VisionConstants.REAR_CAMERA_TO_ROBOT));

    try {
      this.aprilTagFieldLayout =
          new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/AprilTagLayout/2023-chargedup.json");
    } catch (IOException io) {
    }

    this.robotPoseEstimator =
        new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);
  }

  /**
   * Returns the latest camera result from the front camera.
   */
  public PhotonPipelineResult getFrontLatestResult() {
    return frontCamera.getLatestResult();
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
    return this.getFrontLatestResult().getTargets();
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
   * Returns the best visible target.
   */
  public PhotonTrackedTarget getBestTarget() {
    PhotonTrackedTarget rear = this.getRearLatestResult().getBestTarget();
    PhotonTrackedTarget front = this.getFrontLatestResult().getBestTarget();
    if (rear.getPoseAmbiguity() < front.getPoseAmbiguity()) {
      return rear;
    } else {
      return front;
    }
  }

  /**
   * Returns the best visible target's ID.
   */
  public Integer getBestFiducialID() {
    return this.getBestTarget().getFiducialId();
  }

  /**
   * Estimates the global field pose based on previous pose and an {@link RobotPoseEstimator}.
   */
  public Optional<Pair<Pose3d, Double>> getEstimatedGlobalPose() {
    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      return Optional.of(new Pair<Pose3d, Double>(result.get().getFirst(), currentTime - result.get().getSecond()));
    } else {
      return Optional.empty();
    }
  }


}
