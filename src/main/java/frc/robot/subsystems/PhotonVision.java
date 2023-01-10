// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonVision extends SubsystemBase {
  private PhotonCamera camera;
  private AprilTagFieldLayout aprilTagFieldLayout;

  /** Creates a new Photonvision. */
  public PhotonVision() {
    this.camera = new PhotonCamera("photonvision");

    try {
      this.aprilTagFieldLayout = new AprilTagFieldLayout("AprilTagLayout/2023-chargedup.json");
    } catch (IOException io) {
    }

  }

  public PhotonPipelineResult getLatestResult() {
    return camera.getLatestResult();
  }

  public List<PhotonTrackedTarget> getTargets() {
    return this.getLatestResult().getTargets();
  }

  public List<Integer> getFiducialIDs() {
    List<Integer> ids = new ArrayList<>();
    for (PhotonTrackedTarget target : this.getTargets()) {
      ids.add(target.getFiducialId());
    }
    return ids;
  }

  public PhotonTrackedTarget getBestTarget() {
    return this.getLatestResult().getBestTarget();
  }

  public Integer getBestFiducialID() {
    return this.getBestTarget().getFiducialId();
  }

  public Pose3d getFieldPose() {
    PhotonTrackedTarget target = this.getBestTarget();
    return PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(),
        aprilTagFieldLayout.getTagPose(target.getFiducialId()), new Transform3d());
  }

}
