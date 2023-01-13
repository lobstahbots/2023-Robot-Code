// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveBase;

/**
 * Drives a {@link DriveBase} until it is in line with the Y position of a given Pose2d.
 */
public class DriveToLineYCommand extends DriveCommand {
  private final Pose2d targetPose;

  /**
   * Drives a {@link DriveBase} to line up with the Y position of a Pose2d. Useful for lining up with a goal.
   */
  public DriveToLineYCommand(DriveBase driveBase, Pose2d targetPose) {
    super(driveBase);
    this.targetPose = targetPose;
  }

  @Override
  public void execute() {
    if (driveBase.getYDistanceToPose(targetPose) > FieldConstants.SCORING_ZONE_DEADBAND) {
      driveBase.tankDrive(AutonConstants.SIMPLE_AUTON_SPEED, AutonConstants.SIMPLE_AUTON_SPEED, false);
    } else if (driveBase.getHeading().minus(targetPose.getRotation()).getDegrees() > DriveConstants.TURN_DEADBAND) {
      CommandScheduler.getInstance().schedule(new TurnCommand(driveBase, targetPose.getRotation().getDegrees()));
    } else {
      this.cancel();
    }
  }

  public boolean isFinished() {
    return false;
  }
}
