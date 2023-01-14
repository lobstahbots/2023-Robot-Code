// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveBase;

/**
 * Makes an {@link DriveBase} follow a path to a target Pose2d.
 */
public class GeneratePathCommand extends DriveCommand {
  private final Pose2d targetPose;
  private final ArrayList<Pose2d> waypoints;

  /**
   * Generates and drives an {@link DriveBase} along a PathPlannerTrajectory to a target Pose2d.
   */
  public GeneratePathCommand(DriveBase driveBase, Pose2d targetPose) {
    super(driveBase);
    this.targetPose = targetPose;
    this.waypoints = new ArrayList<Pose2d>();
  }

  /**
   * Generates and drives an {@link DriveBase} along a PathPlannerTrajectory through a set of provided waypoints to a
   * target Pose2d.
   */
  public GeneratePathCommand(DriveBase driveBase, Pose2d targetPose, ArrayList<Pose2d> waypoints) {
    super(driveBase);
    this.targetPose = targetPose;
    this.waypoints = waypoints;
  }

  public void execute() {
    PathPlannerTrajectory trajectory = driveBase.generatePath(this.targetPose, this.waypoints);
    CommandScheduler.getInstance().schedule(new PathFollowCommand(this.driveBase, trajectory, false, true));
  }

  public boolean isFinished() {
    return false;
  }
}
