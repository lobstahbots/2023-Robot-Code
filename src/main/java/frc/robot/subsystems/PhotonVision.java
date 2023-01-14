// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

/**
 * A subsystem that controls the PhotonVision tracking on the robot.
 */
public class PhotonVision extends SubsystemBase {
  private PhotonCamera camera;
  private AprilTagFieldLayout aprilTagFieldLayout;
  private RobotPoseEstimator robotPoseEstimator;

  private ArrayList<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<>();

  /** Constructs a new Photonvision. */
  public PhotonVision() {
    this.camera = new PhotonCamera("photonvision");
    camList.add(new Pair<PhotonCamera, Transform3d>(camera, VisionConstants.CAMERA_TO_ROBOT));
    this.robotPoseEstimator =
        new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);


    try {
      this.aprilTagFieldLayout = new AprilTagFieldLayout("AprilTagLayout/2023-chargedup.json");
    } catch (IOException io) {
    }

  }

  /**
   * Returns the latest camera result.
   */
  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  /**
   * Returns a List of the visible AprilTags.
   */
  public List<PhotonTrackedTarget> getTargets() {
    return this.getLatestResult().getTargets();
  }

  /**
   * Returns a List of the IDs of the visible AprilTags.
   */
  public List<Integer> getFiducialIDs() {
    List<Integer> ids = new ArrayList<>();
    for (PhotonTrackedTarget target : this.getTargets()) {
      ids.add(target.getFiducialId());
    }
    return ids;
  }

  /**
   * Returns the best visible target.
   */
  public PhotonTrackedTarget getBestTarget() {
    return this.getLatestResult().getBestTarget();
  }

  /**
   * Returns the best visible target's ID.
   */
  public Integer getBestFiducialID() {
    return this.getBestTarget().getFiducialId();
  }

  /**
   * Estimates the field pose based on a {@link Transform3d} to an AprilTag.
   */
  public Pose3d getFieldPose() {
    PhotonTrackedTarget target = this.getBestTarget();
    Optional<Pose3d> tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId());
    return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        tagPose.get(), VisionConstants.CAMERA_TO_ROBOT);
  }

  /**
   * Estimates the global field pose based on previous pose and an {@link RobotPoseEstimator}.
   */
  public Pair<Pose3d, Double> getEstimatedGlobalPose() {
    double currentTime = Timer.getFPGATimestamp();
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      return new Pair<Pose3d, Double>(result.get().getFirst(), currentTime - result.get().getSecond());
    } else {
      return new Pair<Pose3d, Double>(null, 0.0);
    }
  }


}
