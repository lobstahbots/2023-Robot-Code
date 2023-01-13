// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.DriveBase;

/**
 * Makes an {@link DriveBase} follow a path to a target Pose2d.
 */
public class GeneratePathCommand extends DriveCommand {
  private final Pose2d targetPose;

  /**
   * Generates and drives the {@link DriveBase} along a PathPlannerTrajectory to a target Pose2d.
   */
  public GeneratePathCommand(DriveBase driveBase, Pose2d targetPose) {
    super(driveBase);
    this.targetPose = targetPose;
  }

  public void execute() {
    PathPlannerTrajectory trajectory = driveBase.generatePath(this.targetPose);
    CommandScheduler.getInstance().schedule(new PathFollowCommand(this.driveBase, trajectory, true, true));
  }

  public boolean isFinished() {
    return false;
  }
}
