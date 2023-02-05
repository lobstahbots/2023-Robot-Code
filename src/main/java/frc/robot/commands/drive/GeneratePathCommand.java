// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveBase;

/**
 * Makes an {@link DriveBase} follow a path to a target Pose2d.
 */
public class GeneratePathCommand extends DriveCommand {
  private final List<Pose2d> waypoints;

  /**
   * Generates and drives an {@link DriveBase} along a PathPlannerTrajectory to a target Pose2d.
   */
  public GeneratePathCommand(DriveBase driveBase, Pose2d targetPose) {
    super(driveBase);
    this.waypoints = new ArrayList<Pose2d>();
    this.waypoints.add(targetPose);
  }

  /**
   * Generates and drives an {@link DriveBase} along a PathPlannerTrajectory through a set of provided waypoints to a
   * target Pose2d.
   */
  public GeneratePathCommand(DriveBase driveBase, List<Pose2d> waypoints) {
    super(driveBase);
    this.waypoints = waypoints;
  }

  public void execute() {
    PathPlannerTrajectory trajectory = driveBase.generatePath(this.waypoints);
    CommandScheduler.getInstance().schedule(new PathFollowCommand(this.driveBase, trajectory, false, true));
  }

  public boolean isFinished() {
    return false;
  }
}
