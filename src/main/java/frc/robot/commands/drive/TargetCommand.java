// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveBase;

/**
 * Drives a {@link DriveBase} through predetermined waypoints to a target to score.
 */
public class TargetCommand extends DriveCommand {
  private final Pose2d targetPose;

  /**
   * Drives a {@link DriveBase} through a trajectory of waypoints to reach a scoring target.
   */
  public TargetCommand(DriveBase driveBase, Pose2d targetPose) {
    super(driveBase);
    this.targetPose = targetPose;
  }

  @Override
  public void execute() {
    /* Finding the waypoint closest to the target. */
    int finalWaypointIndex = 0;
    for (int i = 0; i < FieldConstants.TRAVELING_WAYPOINTS.length; i++) {
      if (Math
          .abs(driveBase.getYDistanceToPose(targetPose, FieldConstants.TRAVELING_WAYPOINTS[i])) < Math
              .abs(driveBase
                  .getYDistanceToPose(targetPose, FieldConstants.TRAVELING_WAYPOINTS[finalWaypointIndex]))) {
        finalWaypointIndex = i;
      }
    }
    /*
     * Finding the starting waypoint (closest to robot) and generating a path to the final waypoint. Logic is slightly
     * different depending on direction the robot is traveling.
     */
    if (driveBase.getYDistanceToPose(driveBase.getPose(), targetPose) < 0) {
      int index = 0;
      while (driveBase.getYDistanceToPose(driveBase.getPose(), FieldConstants.TRAVELING_WAYPOINTS[index]) > 0) {
        index++;
      }
      index++;
      ArrayList<Pose2d> waypoints = new ArrayList<>();
      for (int i = index; i < finalWaypointIndex; i++) {
        waypoints.add(new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(90)));
      }
      CommandScheduler.getInstance()
          .schedule(new GeneratePathCommand(driveBase, targetPose, waypoints));

    } else {
      int index = FieldConstants.TRAVELING_WAYPOINTS.length - 1;
      while (driveBase.getYDistanceToPose(driveBase.getPose(), FieldConstants.TRAVELING_WAYPOINTS[index]) < 0) {
        index--;
      }
      index--;
      ArrayList<Pose2d> waypoints = new ArrayList<>();
      for (int i = index; i >= finalWaypointIndex; i--) {
        waypoints.add(new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(-90)));
      }
      CommandScheduler.getInstance()
          .schedule(new GeneratePathCommand(driveBase, targetPose,
              waypoints));
    }
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
