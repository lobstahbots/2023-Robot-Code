/*
 * MIT License
 *
 * Copyright (c) 2022 PhotonVision
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
 * WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
 * OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import lobstah.stl.math.LobstahMath;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * The PhotonPoseEstimator class filters or combines readings from all the AprilTags visible at a given timestamp on the
 * field to produce a single robot in field pose, using the strategy set below. Example usage can be found in our
 * apriltagExample example project.
 */
public class PhotonPoseEstimator {

  private AprilTagFieldLayout fieldTags;
  private final PhotonCamera camera;
  private final Transform3d robotToCamera;

  private final Set<Integer> reportedErrors = new HashSet<>();

  public final int id;

  /**
   * Create a new PhotonPoseEstimator.
   *
   * @param fieldTags A WPILib {@link AprilTagFieldLayout} linking AprilTag IDs to Pose3d objects with respect to the
   *          FIRST field
   * @param strategy The strategy it should use to determine the best pose.
   * @param camera PhotonCameras and
   * @param robotToCamera Transform3d from the center of the robot to the camera mount positions (ie, robot ➔ camera)
   */
  public PhotonPoseEstimator(
      AprilTagFieldLayout fieldTags,
      PhotonCamera camera,
      Transform3d robotToCamera, int id) {
    this.fieldTags = fieldTags;
    this.camera = camera;
    this.robotToCamera = robotToCamera;
    this.id = id;
  }

  /**
   * Get the AprilTagFieldLayout being used by the PositionEstimator.
   *
   * @return the AprilTagFieldLayout
   */
  public AprilTagFieldLayout getFieldTags() {
    return fieldTags;
  }

  /**
   * Set the AprilTagFieldLayout being used by the PositionEstimator.
   *
   * @param fieldTags the AprilTagFieldLayout
   */
  public void setFieldTags(AprilTagFieldLayout fieldTags) {
    this.fieldTags = fieldTags;
  }

  /**
   * Poll data from the configured cameras and update the estimated position of the robot. Returns empty if there are no
   * cameras set or no targets were found from the cameras.
   *
   * @return an EstimatedRobotPose with an estimated pose, and information about the camera(s) and pipeline results used
   *         to create the estimate
   */
  public Optional<EstimatedRobotPose> update() {
    if (camera == null) {
      DriverStation.reportError("[PhotonPoseEstimator] Missing camera!", false);
      return Optional.empty();
    }

    PhotonPipelineResult cameraResult = camera.getLatestResult();
    if (!cameraResult.hasTargets()) {
      return Optional.empty();
    }

    return lowestAmbiguityStrategy(cameraResult);
  }

  /**
   * Return the estimated position of the robot with the lowest position ambiguity from a List of pipeline results.
   *
   * @param result pipeline result
   * @return the estimated position of the robot in the FCS and the estimated timestamp of this estimation.
   */
  private Optional<EstimatedRobotPose> lowestAmbiguityStrategy(PhotonPipelineResult result) {
    PhotonTrackedTarget lowestAmbiguityTarget = null;

    double lowestAmbiguityScore = 10;

    for (PhotonTrackedTarget target : result.targets) {
      double targetPoseAmbiguity = target.getPoseAmbiguity();
      // Make sure the target is a Fiducial target.
      if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
        lowestAmbiguityScore = targetPoseAmbiguity;
        lowestAmbiguityTarget = target;
      }
    }

    // Although there are confirmed to be targets, none of them may be fiducial
    // targets.
    if (lowestAmbiguityTarget == null)
      return Optional.empty();

    int targetFiducialId = lowestAmbiguityTarget.getFiducialId();

    Optional<Pose3d> targetPosition = fieldTags.getTagPose(targetFiducialId);

    if (targetPosition.isEmpty()) {
      reportFiducialPoseError(targetFiducialId);
      return Optional.empty();
    }

    if (lowestAmbiguityScore > 0.2) {
      return Optional.of(
          new EstimatedRobotPose(
              targetPosition
                  .get()
                  .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                  .transformBy(robotToCamera.inverse()).toPose2d(),
              result.getTimestampSeconds(), 0));
    } else {
      double confidence = LobstahMath.scaleNumberToRange(0.2 - lowestAmbiguityScore, 0, 0.2, 0, 1);
      return Optional.of(
          new EstimatedRobotPose(
              targetPosition
                  .get()
                  .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                  .transformBy(robotToCamera.inverse()).toPose2d(),
              result.getTimestampSeconds(), confidence));
    }
  }

  /**
   * Difference is defined as the vector magnitude between the two poses
   *
   * @return The absolute "difference" (>=0) between two Pose3ds.
   */
  public double calculateDifference(Pose3d x, Pose3d y) {
    return x.getTranslation().getDistance(y.getTranslation());
  }

  private void reportFiducialPoseError(int fiducialId) {
    if (!reportedErrors.contains(fiducialId)) {
      DriverStation.reportError(
          "[PhotonPoseEstimator] Tried to get pose of unknown AprilTag: " + fiducialId, false);
      reportedErrors.add(fiducialId);
    }
  }
}
