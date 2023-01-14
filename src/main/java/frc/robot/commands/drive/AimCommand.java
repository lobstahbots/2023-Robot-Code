// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import java.util.ArrayList;

import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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
    SmartDashboard.putNumber("Distance TO POse", driveBase.getYDistanceToPose(driveBase.getPose(), targetPose));
    int finalWayPoint = 0;
    for (int i = 0; i < FieldConstants.TRAVELING_WAYPOINTS.length; i++) {
      if (Math
          .abs(driveBase.getYDistanceToPose(targetPose, FieldConstants.TRAVELING_WAYPOINTS[i])) < Math
              .abs(driveBase
                  .getYDistanceToPose(targetPose, FieldConstants.TRAVELING_WAYPOINTS[finalWayPoint]))) {
        finalWayPoint = i;
      }
    }
    if (driveBase.getYDistanceToPose(driveBase.getPose(), targetPose) < 0) {
      int index = 0;
      while (driveBase.getYDistanceToPose(driveBase.getPose(), FieldConstants.TRAVELING_WAYPOINTS[index]) > 0) {
        index++;
      }
      SmartDashboard.putNumber("Index", index);
      SmartDashboard.putNumber("Final Way Point", finalWayPoint);
      ArrayList<Pose2d> waypoints = new ArrayList<>();
      for (int i = finalWayPoint; i >= index; i--) {
        waypoints.add(new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(90)));
        SmartDashboard.putString("Index" + String.valueOf(i), new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(90)).toString());
      }

      // CommandScheduler.getInstance()
      // .schedule(new GeneratePathCommand(driveBase, FieldConstants.TRAVELING_WAYPOINTS[finalWayPoint], waypoints));
    } else {
      int index = FieldConstants.TRAVELING_WAYPOINTS.length - 1;
      while (driveBase.getYDistanceToPose(driveBase.getPose(), FieldConstants.TRAVELING_WAYPOINTS[index]) < 0) {
        index--;
      }
      SmartDashboard.putNumber("Index", index);
      SmartDashboard.putNumber("Final Way Point", finalWayPoint);
      ArrayList<Pose2d> waypoints = new ArrayList<>();
      for (int i = index; i < finalWayPoint; i++) {
        waypoints.add(new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(-90)));
        SmartDashboard.putString("Index" + String.valueOf(i), new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(-90)).toString());
      }
      // CommandScheduler.getInstance()
      // .schedule(new GeneratePathCommand(driveBase, FieldConstants.TRAVELING_WAYPOINTS[index]));
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
