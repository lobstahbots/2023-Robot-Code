// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.ArrayList;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.drive.PathFollowCommand;
import frc.robot.commands.drive.StraightDriveCommand;
import frc.robot.subsystems.DriveBase;
import lobstah.stl.command.TimedCommand;

/**
 * A class that generates autonomous routines for the robot.
 */
public class AutonGenerator {

  private final DriveBase driveBase;

  /**
   * Constructs an AutonGenerator with a {@link DriveBase}.
   *
   * @param driveBase The drivetrain for the AutonGenerator to control.
   */
  public AutonGenerator(DriveBase driveBase) {
    this.driveBase = driveBase;
  }

  /**
   * Creates and returns a simple autonomous routine to score a preload and drive across the line.
   */
  public Command getSimpleAutonCommand() {
    final Command simpleAutonCommand =
        new TimedCommand(
            AutonConstants.SIMPLE_AUTON_RUNTIME,
            new StraightDriveCommand(
                driveBase,
                AutonConstants.SIMPLE_AUTON_SPEED, false));
    return simpleAutonCommand;
  }

  /**
   * Returns a command to follow a path.
   * 
   * @param initialPosition The starting position of the robot
   * @param crossingPosition Where the robot crosses out of the Community.
   * @param finalPosition Which game element the path ends at.
   */
  public Command getPathFollowCommand(int initialPosition, int crossingPosition, int finalPosition) {
    ArrayList<PathPlannerTrajectory> pathGroup = this.getPath(initialPosition, crossingPosition, finalPosition);
    return new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // driveBase.resetOdometry(pathGroup.get(0).getInitialPose().getTranslation(),
        // pathGroup.get(0).getInitialPose().getRotation());
        // }),
        new PathFollowCommand(this.driveBase, pathGroup.get(0)),
        new PathFollowCommand(this.driveBase, pathGroup.get(1)));
  }

  /**
   * Constructs and returns a {@link PathPlannerTrajectory} to follow based on provided starting, crossing, and ending
   * position.
   * 
   * @param initialPosition The starting position of the robot
   * @param crossingPosition Where the robot crosses out of the Community.
   * @param finalPosition Which game element the path ends at.
   */
  public ArrayList<PathPlannerTrajectory> getPath(int initialPosition, int crossingPosition, int finalPosition) {
    ArrayList<PathPlannerTrajectory> pathGroup = new ArrayList<>();
    String firstPathName = String.valueOf(initialPosition) + "-" + String.valueOf(crossingPosition);
    String secondPathName = "_" + String.valueOf(crossingPosition) + "-" + String.valueOf(finalPosition);
    PathPlannerTrajectory firstPath = PathPlanner.loadPath(firstPathName,
        new PathConstraints(PathConstants.MAX_DRIVE_SPEED, PathConstants.MAX_ACCELERATION));
    PathPlannerTrajectory secondPath = PathPlanner.loadPath(secondPathName,
        new PathConstraints(PathConstants.MAX_DRIVE_SPEED, PathConstants.MAX_ACCELERATION));
    if (driveBase.getDistanceToPose(firstPath.getInitialPose()).getTranslation()
        .getNorm() > PathConstants.MAX_OFFSET_START) {
      SmartDashboard.putNumber("Distance from start",
          driveBase.getDistanceToPose(firstPath.getInitialPose()).getTranslation()
              .getNorm());
      return pathGroup;
    }
    pathGroup.add(firstPath);
    pathGroup.add(secondPath);

    return pathGroup;

  }

}
