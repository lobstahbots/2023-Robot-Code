// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.ArrayList;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmPose;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.ArmPresets;
import frc.robot.commands.arm.ArmToPoseCommand;
import frc.robot.commands.arm.ArmToPoseWithRetractionCommand;
import frc.robot.commands.arm.ArmTowardsPoseCommand;
import frc.robot.commands.arm.ArmTowardsPoseWithRetractionCommand;
import frc.robot.commands.drive.PathFollowCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.StraightDriveCommand;
import frc.robot.commands.drive.TurnToAngleCommand;
import frc.robot.commands.intake.SpinIntakeCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import lobstah.stl.command.ConstructLaterCommand;
import lobstah.stl.command.TimedCommand;

/**
 * A class that generates autonomous routines for the robot.
 */
public class AutonGenerator {

  private final DriveBase driveBase;
  private final Arm arm;
  private final Intake intake;

  /**
   * Constructs an AutonGenerator with a {@link DriveBase}.
   *
   * @param driveBase The drivetrain for the AutonGenerator to control.
   */
  public AutonGenerator(DriveBase driveBase, Arm arm, Intake intake) {
    this.driveBase = driveBase;
    this.arm = arm;
    this.intake = intake;
  }


  public Command getDoNothingCommand() {
    return new StopDriveCommand(driveBase);
  }

  /**
   * Creates and returns an autonomous routine to score a preload based on row number, then drive following a path.
   * 
   * @param row A supplier for the goal row. 0 -> high goal, 1 -> mid goal, 2 -> low goal
   * @param initialPosition The starting position of the robot
   * @param crossingPosition Where the robot crosses out of the Community.
   * @param finalPosition Which game element the path ends at.
   */
  public Command getScoreAndDriveCommand(int row, int initialPosition, int crossingPosition, int finalPosition) {
    return getScoreCommand(row).andThen(new WaitCommand(0.5))
        .andThen(
            getPathFollowCommand(initialPosition, crossingPosition, finalPosition));
    // .andThen(new ConstructLaterCommand(() -> getGroundPickupCommand(1, 0, 0)))
    // .andThen(new ArmToPoseWithRetractionCommand(arm, ArmPresets.STOWED, 1));
    // .andThen(new ConstructLaterCommand(() -> getReturnCommand(1, 0)));
  }

  public Command getGroundPickupCommand(int scorePiece, int finalPosition, int row) {
    return new ParallelRaceGroup(
        new ConstructLaterCommand(
            () -> new PathFollowCommand(driveBase,
                driveBase.generatePath(
                    driveBase.flipWaypointBasedOnAlliance(FieldConstants.GROUND_PICKUP_POSES[scorePiece], true)))
                        .andThen(new WaitCommand(0.5))),
        new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.GROUND_PICKUP),
        new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE));
  }

  public Command getReturnCommand(int finalPosition, int row) {
    return new TimedCommand(1,
        new TurnToAngleCommand(driveBase,
            driveBase.flipWaypointBasedOnAlliance(FieldConstants.SCORING_WAYPOINTS[finalPosition], true).getRotation(),
            1)).andThen(
                new ConstructLaterCommand(() -> new PathFollowCommand(driveBase,
                    driveBase.generatePath(
                        driveBase.flipWaypointBasedOnAlliance(FieldConstants.RETURNING_CROSSING_WAYPOINTS[0], true)))))
                .andThen(new TimedCommand(0.5,
                    new TurnToAngleCommand(driveBase, FieldConstants.SCORING_WAYPOINTS[finalPosition].getRotation(),
                        1)))
                .andThen(new ConstructLaterCommand(() -> new PathFollowCommand(driveBase,
                    driveBase.generatePath(driveBase
                        .flipWaypointBasedOnAlliance(FieldConstants.ENTERING_SCORING_ZONE_WAYPOINTS[0], true)))))
                .andThen(new ConstructLaterCommand(
                    () -> getPathToTargetCommand(driveBase,
                        () -> driveBase.flipWaypointBasedOnAlliance(FieldConstants.SCORING_WAYPOINTS[finalPosition],
                            true))))
                .andThen(new TimedCommand(0.5,
                    new TurnToAngleCommand(driveBase,
                        driveBase.flipWaypointBasedOnAlliance(FieldConstants.SCORING_WAYPOINTS[finalPosition], true)
                            .getRotation(),
                        1)))
                .andThen(getScoreCommand(row));
  }

  /**
   * Creates and returns a simple autonomous routine to score a preload based on row number.
   * 
   * @param rowSupplier A supplier for the goal row. 0 -> high goal, 1 -> mid goal, 2 -> low goal
   */
  public Command getScoreCommand(Supplier<Integer> rowSupplier) {
    if (rowSupplier.get() == 0) {
      return getScoreCommand(ArmPresets.HIGH_GOAL_SCORING, true);
    } else if (rowSupplier.get() == 1) {
      return getScoreCommand(ArmPresets.MID_GOAL_SCORING, true);
    } else if (rowSupplier.get() == 2) {
      return getScoreCommand(ArmPresets.LOW_GOAL_SCORING, false);
    } else {
      return new InstantCommand();
    }
  }

  /**
   * Creates and returns a simple autonomous routine to score a preload based on row number.
   * 
   * @param row The goal row. 0 -> high goal, 1 -> mid goal, 2 -> low goal
   */
  public Command getScoreCommand(int row) {
    if (row == 0) {
      return getScoreCommand(ArmPresets.HIGH_GOAL_SCORING, true);
    } else if (row == 1) {
      return getScoreCommand(ArmPresets.MID_GOAL_SCORING, true);
    } else if (row == 2) {
      return getScoreCommand(ArmPresets.LOW_GOAL_SCORING, false);
    } else {
      return new InstantCommand();
    }
  }

  /**
   * Creates and returns a simple autonomous routine to score a preload at a given {@link ArmPose}.
   * 
   * @param position The ArmPose to score at.
   * @param placeDown Whether or not to move the arm downwards while scoring.
   */
  public Command getScoreCommand(ArmPose position, boolean placeDown) {
    return new ArmToPoseWithRetractionCommand(arm, position,
        AutonConstants.AUTON_SCORING_TOLERANCE)
            .andThen(new ArmToPoseCommand(arm,
                position.translateBy(ArmPresets.CONE_SCORING_DROPDOWN),
                AutonConstants.AUTON_SCORING_TOLERANCE).unless(() -> !placeDown))
            .andThen(
                new ParallelRaceGroup(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE).asProxy(),
                    new TimedCommand(AutonConstants.OUTTAKE_RUNTIME,
                        new ArmToPoseCommand(arm,
                            position.translateBy(ArmPresets.CONE_SCORING_BACKOFF),
                            AutonConstants.AUTON_SCORING_TOLERANCE))))
            .andThen(
                new ParallelRaceGroup(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE).asProxy(),
                    new ArmToPoseWithRetractionCommand(arm, ArmPresets.STOWED,
                        AutonConstants.AUTON_SCORING_TOLERANCE)))
            .andThen(new TimedCommand(
                AutonConstants.DRIVE_BACK_TIME,
                new StraightDriveCommand(
                    driveBase,
                    AutonConstants.DRIVE_BACK_SPEED, false)));
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
   * Creates and returns a command to path to a target side of the Driver Station and pick up a game piece.
   * 
   * @param targetPose The position to drive to.
   */
  public Command getDriveToPlayerStationCommand(Pose2d targetPose) {
    Pose2d waypoint = driveBase.flipWaypointBasedOnAlliance(
        new Pose2d(targetPose.getX() - 1, targetPose.getY(), targetPose.getRotation()), true);
    Pose2d flippedTargetPose = driveBase.flipWaypointBasedOnAlliance(targetPose, true);
    return new ParallelRaceGroup(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE),
        new SequentialCommandGroup( // Drive to waypoint, then turn while raising arm
            new ConstructLaterCommand(() -> new PathFollowCommand(driveBase, driveBase.generatePath(waypoint))),
            new ParallelDeadlineGroup(new ArmToPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP, 5),
                new TurnToAngleCommand(driveBase, flippedTargetPose.getRotation(), 1)),
            new ParallelRaceGroup( // Maintain arm angle and spin intake
                new SequentialCommandGroup( // Meanwhile, drive to target and turn briefly to ensure correct angle
                    new ConstructLaterCommand(
                        () -> new PathFollowCommand(driveBase, driveBase.generatePath(flippedTargetPose))),
                    new TimedCommand(0.2, new TurnToAngleCommand(driveBase, flippedTargetPose.getRotation(), 1))),
                new ArmTowardsPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP))
                    .andThen(new ParallelRaceGroup(new TimedCommand(0.25, new StopDriveCommand(driveBase)),
                        new ArmTowardsPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP))) // Hold for a second
                    .andThen(new TimedCommand(1.0, new ParallelCommandGroup(
                        new ArmTowardsPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP),
                        new StraightDriveCommand(driveBase, -0.3, false)))))) // Drive away with arm raised still
                            .unless(() -> Math.abs(
                                driveBase.getDistanceToPose(flippedTargetPose)
                                    .getX()) > FieldConstants.MAX_PLAYER_STATION_X_ZONE
                                || Math.abs(driveBase.getDistanceToPose(flippedTargetPose)
                                    .getY()) > FieldConstants.MAX_PLAYER_STATION_Y_ZONE);
  }

  /**
   * Returns a command to follow a path.
   * 
   * @param initialPosition The starting position of the robot
   * @param crossingPosition Where the robot crosses out of the Community.
   * @param finalPosition Which game element the path ends at.
   */
  public Command getPathFollowCommand(int initialPosition, int crossingPosition, int finalPosition) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      initialPosition = 8 - initialPosition;
    }

    if (initialPosition <= 2) {
      crossingPosition = 0;
    } else if (initialPosition >= 6) {
      crossingPosition = 1;
    }
    Pose2d crossingPose =
        driveBase.flipWaypointBasedOnAlliance(FieldConstants.CROSSING_WAYPOINTS[crossingPosition], true);

    return new SequentialCommandGroup(
        new ConstructLaterCommand(() -> getPathToTargetCommand(driveBase, () -> crossingPose)),
        new TurnToAngleCommand(driveBase, crossingPose.getRotation(),
            PathConstants.TURN_ANGLE_DEADBAND));
    // new ConstructLaterCommand(() -> getPathToTargetCommand(driveBase, () -> crossingPose)));
    // new ConstructLaterCommand(() -> new PathFollowCommand(driveBase,
    // driveBase.generatePath(FieldConstants.ENDING_AUTON_POSES[finalPosition]))));
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
    pathGroup.add(firstPath);
    pathGroup.add(secondPath);

    return pathGroup;

  }

  public Command getPathToTargetCommand(DriveBase driveBase, Supplier<Pose2d> targetSupplier) {
    Pose2d targetPose = targetSupplier.get();
    /* Finding the waypoint closest to the target. */
    int finalWaypointIndex = 0;
    for (int i = 0; i < FieldConstants.TRAVELING_WAYPOINTS.length; i++) {
      if (Math.abs(targetPose.getY() - FieldConstants.TRAVELING_WAYPOINTS[i].getY()) < Math
          .abs(targetPose.getY() - FieldConstants.TRAVELING_WAYPOINTS[finalWaypointIndex].getY())) {
        finalWaypointIndex = i;
      }
    }
    /*
     * Finding the starting waypoint (closest to robot) and generating a path to the final waypoint. Logic is slightly
     * different depending on direction the robot is traveling.
     */
    ArrayList<Pose2d> waypoints = new ArrayList<>();
    int index = 0;
    if (driveBase.getDistanceToPose(targetPose).getY() < 0) {
      while (driveBase.getDistanceToPose(FieldConstants.TRAVELING_WAYPOINTS[index]).getY() > 0) {
        if (index >= FieldConstants.TRAVELING_WAYPOINTS.length - 1) {
          index = FieldConstants.TRAVELING_WAYPOINTS.length - 1;
          break;
        }
        index++;
      }
    } else {
      index = FieldConstants.TRAVELING_WAYPOINTS.length - 1;
      while (driveBase.getDistanceToPose(FieldConstants.TRAVELING_WAYPOINTS[index]).getY() < 0) {
        if (index == 0) {
          index = 0;
          break;
        }
        index--;
      }
    }
    if (finalWaypointIndex > index) { // Traverse indexes
      index = MathUtil.clamp(index + 1, 0, FieldConstants.TRAVELING_WAYPOINTS.length - 1);
      for (int i = index; i < finalWaypointIndex; i++) {
        waypoints.add(driveBase.flipWaypointBasedOnAlliance(new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(90)), false));
      }
    } else {
      index = MathUtil.clamp(index - 1, 0, FieldConstants.TRAVELING_WAYPOINTS.length - 1);
      for (int i = index; i >= finalWaypointIndex; i--) {
        waypoints.add(driveBase.flipWaypointBasedOnAlliance(new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[i].getX(),
            FieldConstants.TRAVELING_WAYPOINTS[i].getY(), Rotation2d.fromDegrees(-90)), false));
      }
    }

    if (waypoints.size() <= 0) {
      return new PathFollowCommand(driveBase, driveBase.generatePath(targetPose));
    }

    return new PathFollowCommand(driveBase, driveBase.generatePath(waypoints))
        .andThen(new TurnToAngleCommand(driveBase, targetPose.getRotation(), PathConstants.TURN_ANGLE_DEADBAND))
        .andThen(
            new ConstructLaterCommand(() -> new PathFollowCommand(driveBase, driveBase.generatePath(targetPose)))
                .andThen(
                    new TurnToAngleCommand(driveBase, targetPose.getRotation(), PathConstants.TURN_ANGLE_DEADBAND)));
  }

}
