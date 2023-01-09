// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.commands.drive.PathFollowCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.StraightDriveCommand;
import lobstah.stl.command.TimedCommand;

/**
 * A subsystem that generates autonomous routines for the robot.
 */
public class AutonGenerator extends SubsystemBase {

  private final DriveBase driveBase;
  private final Command doNothingCommand;

  /**
   * Constructs an AutonGenerator with a {@link DriveBase}.
   *
   * @param driveBase The drivetrain for the AutonGenerator to control.
   */
  public AutonGenerator(DriveBase driveBase) {
    this.driveBase = driveBase;
    this.doNothingCommand = new StopDriveCommand(driveBase);
  }

  /**
   * Returns a default autonomous command to do nothing.
   */
  public Command getDefaultCommand() {
    return this.getDoNothingCommand();
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
   * Returns an autonomous command to do nothing.
   */
  public Command getDoNothingCommand() {
    return this.doNothingCommand;
  }

  /**
   * Returns a command to follow a path.
   * 
   * @param initialPosition The starting position of the robot
   * @param crossingPosition Where the robot crosses out of the Community.
   * @param finalPosition Which game element the path ends at.
   */
  public Command getPathFollowCommand(int initialPosition, int crossingPosition, int finalPosition) {
    return new PathFollowCommand(this.driveBase, this.getPath(initialPosition, crossingPosition, finalPosition), true);
  }

  /**
   * Constructs and returns a {@link PathPlannerTrajectory} to follow based on provided starting, crossing, and ending
   * position.
   * 
   * @param initialPosition The starting position of the robot
   * @param crossingPosition Where the robot crosses out of the Community.
   * @param finalPosition Which game element the path ends at.
   */
  public PathPlannerTrajectory getPath(int initialPosition, int crossingPosition, int finalPosition) {
    PathPlannerTrajectory path = PathPlanner.loadPath("New Path",
        new PathConstraints(PathConstants.MAX_DRIVE_SPEED, PathConstants.MAX_ACCELERATION));
    return path;
  }

}
