// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveBase;

/**
 * Drives an {@link DriveBase} to a given target Pose using point turns.
 */
public class TargetCommand extends SequentialCommandGroup {
  private final Pose2d targetPose;
  private final DriveBase driveBase;

  /**
   * Drives an {@link DriveBase} to a given target Pose using point turns.
   */
  public TargetCommand(DriveBase driveBase, Pose2d targetPose) {
    this.driveBase = driveBase;
    this.targetPose = targetPose;

    if (driveBase.getXDistanceToPose(this.targetPose) > FieldConstants.MAX_AUTO_DISTANCE_METERS) {
      this.cancel();
    }

    if (this.driveBase.getYDistanceToPose(this.driveBase.getPose(),
        this.targetPose) < FieldConstants.SCORING_ZONE_DEADBAND) {
      CommandScheduler.getInstance().schedule(new GeneratePathCommand(driveBase, targetPose));
    } else {
      if (this.driveBase.getYDistanceToPose(this.driveBase.getPose(), this.targetPose) < 0) {
        addCommands(new TurnCommand(driveBase, 90), new DriveToLineYCommand(driveBase, targetPose));
      } else {

      }
    }
  }
}
