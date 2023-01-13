// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.FieldConstants;
import frc.robot.subsystems.DriveBase;

public class AimCommand extends DriveCommand {
  /** Creates a new AimCommand. */
  private final Pose2d targetPose;

  public AimCommand(DriveBase driveBase, Pose2d targetPose) {
    super(driveBase);
    this.targetPose = targetPose;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("Distance TO POse", driveBase.getYDistanceToPose(targetPose));
    if (driveBase.getYDistanceToPose(targetPose) < 0) {
      int index = 0;
      while (driveBase.getYDistanceToPose(FieldConstants.TRAVELING_RIGHT_WAYPOINTS[index]) < 0) {
        index++;
      }
      SmartDashboard.putNumber("Index", index);
      // CommandScheduler.getInstance()
      // .schedule(new GeneratePathCommand(driveBase, FieldConstants.TRAVELING_RIGHT_WAYPOINTS[index]));
    } else {
      int index = FieldConstants.TRAVELING_LEFT_WAYPOINTS.length - 1;
      while (driveBase.getYDistanceToPose(FieldConstants.TRAVELING_LEFT_WAYPOINTS[index]) > 0) {
        index--;
      }
      SmartDashboard.putNumber("Index", index);
      // CommandScheduler.getInstance()
      // .schedule(new GeneratePathCommand(driveBase, FieldConstants.TRAVELING_LEFT_WAYPOINTS[index]));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
