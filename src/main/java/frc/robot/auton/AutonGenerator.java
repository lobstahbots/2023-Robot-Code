// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auton;

import java.util.ArrayList;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.ArmPose;
import frc.robot.Constants.AutonConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.ArmPresets;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmToPoseCommand;
import frc.robot.subsystems.arm.ArmToPoseWithRetractionCommand;
import frc.robot.subsystems.arm.ArmTowardsPoseCommand;
import frc.robot.subsystems.driveBase.DriveBase;
import frc.robot.subsystems.driveBase.DriveBasePathFollowCommand;
import frc.robot.subsystems.driveBase.DriveBaseStopCommand;
import frc.robot.subsystems.driveBase.DriveBaseStraightCommand;
import frc.robot.subsystems.driveBase.DriveBaseTurnToAngleCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeSpinCommand;
import lobstah.stl.command.ConstructLaterCommand;
import lobstah.stl.command.TimedCommand;

/**
 * A class that generates autonomous routines for the robot.
 */
public class AutonGenerator {

  private final DriveBase driveBase;
  private final Arm arm;
  private final Intake intake;

  public enum Auton {
    SCORE_AND_DRIVE, DRIVE, SCORE, DO_NOTHING
  }

  public enum CrossingPosition {
    LEFT, RIGHT
  }

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
    return new DriveBaseStopCommand(driveBase);
  }

  /**
   * Creates and returns an autonomous routine to score a preload based on row number, then drive following a path.
   * 
   * @param row A supplier for the goal row. 0 -> high goal, 1 -> mid goal, 2 -> low goal
   * @param initialPosition The starting position of the robot
   * @param crossingPosition Where the robot crosses out of the Community.
   * @param finalPosition Which game element the path ends at.
   */
  public Command getScoreAndDriveCommand(int row, int initialPosition, CrossingPosition crossingPosition,
      boolean twoElement,
      int secondElementPosition) {
    if (initialPosition <= 2) {
      crossingPosition = CrossingPosition.RIGHT;
    } else if (initialPosition >= 6) {
      crossingPosition = CrossingPosition.LEFT;
    }

    if (DriverStation.getAlliance() == Alliance.Red) {
      secondElementPosition = 8 - secondElementPosition;
    }

    Pose2d crossingOutPose;
    Pose2d crossingInPose;
    Pose2d pickupPose;

    if (DriverStation.getAlliance() == Alliance.Blue) {
      if (crossingPosition == CrossingPosition.LEFT) {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.EXITING_CROSSING_WAYPOINTS[1], true);
        pickupPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.GROUND_PICKUP_POSES[1], true);
        crossingInPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.ENTERING_SCORING_ZONE_WAYPOINTS[1], true);
      } else {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.EXITING_CROSSING_WAYPOINTS[0], true);
        pickupPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.GROUND_PICKUP_POSES[0], true);
        crossingInPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.ENTERING_SCORING_ZONE_WAYPOINTS[0], true);
      }
    } else {
      if (crossingPosition == CrossingPosition.LEFT) {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.EXITING_CROSSING_WAYPOINTS[0], true);
        pickupPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.GROUND_PICKUP_POSES[0], true);
        crossingInPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.ENTERING_SCORING_ZONE_WAYPOINTS[0], true);
      } else {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.EXITING_CROSSING_WAYPOINTS[1], true);
        pickupPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.GROUND_PICKUP_POSES[1], true);
        crossingInPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.ENTERING_SCORING_ZONE_WAYPOINTS[1], true);
      }
    }

    if (twoElement) {
      return new SequentialCommandGroup(
          getScoreCommand(row),
          getStage1AutonPathCommand(crossingOutPose),
          getGroundPickupCommand(pickupPose),
          getStage2AutonCommand(crossingInPose, secondElementPosition),
          getScoreCommand(row));
    }
    return new SequentialCommandGroup(
        getScoreCommand(row),
        getStage1AutonPathCommand(crossingOutPose));

  }

  public Command getExitCommunityCommand(int initialPosition, CrossingPosition crossingOutPosition) {
    if (initialPosition <= 2) {
      crossingOutPosition = CrossingPosition.RIGHT;
    } else if (initialPosition >= 6) {
      crossingOutPosition = CrossingPosition.LEFT;
    }

    Pose2d crossingOutPose;

    if (DriverStation.getAlliance() == Alliance.Blue) {
      if (crossingOutPosition == CrossingPosition.LEFT) {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.CROSSING_WAYPOINTS[1], true);
      } else {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.CROSSING_WAYPOINTS[0], true);
      }
    } else {
      if (crossingOutPosition == CrossingPosition.LEFT) {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.CROSSING_WAYPOINTS[0], true);
      } else {
        crossingOutPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.CROSSING_WAYPOINTS[1], true);
      }
    }
    return getStage1AutonPathCommand(crossingOutPose);
  }

  /**
   * Creates and returns a simple autonomous routine to score a preload based on row number.
   * 
   * @param row The goal row. 0 -> high goal, 1 -> mid goal, 2 -> low goal
   */
  public Command getScoreCommand(int row) {
    if (row == 0) {
      return composeScoreCommandHelper(ArmPresets.HIGH_GOAL_SCORING, true);
    } else if (row == 1) {
      return composeScoreCommandHelper(ArmPresets.MID_GOAL_SCORING, true);
    } else if (row == 2) {
      return composeScoreCommandHelper(ArmPresets.LOW_GOAL_SCORING, false);
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
  private Command composeScoreCommandHelper(ArmPose position, boolean placeDown) {
    return new ArmToPoseWithRetractionCommand(arm, position,
        2 * AutonConstants.AUTON_SCORING_TOLERANCE)
            .andThen(new ArmToPoseCommand(arm,
                position.translateBy(ArmPresets.CONE_SCORING_DROPDOWN),
                AutonConstants.AUTON_SCORING_TOLERANCE).unless(() -> !placeDown))
            .andThen(
                new ParallelRaceGroup(new IntakeSpinCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE).asProxy(),
                    new ArmToPoseWithRetractionCommand(arm, ArmPresets.STOWED,
                        AutonConstants.AUTON_SCORING_TOLERANCE)));
  }

  /**
   * Creates and returns a simple autonomous routine to score a preload and drive across the line.
   */
  public Command getSimpleAutonCommand() {
    return new TimedCommand(
        AutonConstants.SIMPLE_AUTON_RUNTIME,
        new DriveBaseStraightCommand(
            driveBase,
            AutonConstants.SIMPLE_AUTON_SPEED));
  }

  /**
   * Creates and returns a command to path to a target side of the Driver Station and pick up a game piece.
   * 
   * @param targetPose The position to drive to.
   */
  public Command getDriveToPlayerStationCommand(Pose2d targetPose) {
    Pose2d waypoint = driveBase.flipWaypointBasedOnAlliance(
        new Pose2d(targetPose.getX() - FieldConstants.PLAYER_STATION_PICKUP_ZONE, targetPose.getY(),
            targetPose.getRotation()),
        true);

    Pose2d flippedTargetPose = driveBase.flipWaypointBasedOnAlliance(targetPose, true);
    return new ParallelRaceGroup(new IntakeSpinCommand(intake, IntakeConstants.INTAKE_VOLTAGE),
        new SequentialCommandGroup( // Drive to waypoint, then turn while raising arm
            new ConstructLaterCommand(
                () -> new DriveBasePathFollowCommand(driveBase, driveBase.generatePath(false, 0.8, 0.8, waypoint))),
            new ArmToPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP, 5),
            new ParallelRaceGroup( // Maintain arm angle and drive to target
                new ConstructLaterCommand(
                    () -> new DriveBasePathFollowCommand(driveBase,
                        driveBase.generatePath(false, 1, 1, flippedTargetPose))),
                new ArmTowardsPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP)),
            new ParallelRaceGroup(new TimedCommand(0.25, new DriveBaseStopCommand(driveBase)), // Hold for a second
                new ArmTowardsPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP)),
            new ParallelRaceGroup( // Drive away with arm raised still
                new ArmTowardsPoseCommand(arm, ArmPresets.PLAYER_STATION_PICKUP),
                new TimedCommand(1,
                    new DriveBaseStraightCommand(driveBase, AutonConstants.DRIVE_BACK_SPEED, false))),
            new ArmToPoseCommand(arm, ArmPresets.STOWED, 1)))
                .unless(() -> Math.abs(
                    driveBase.getDistanceToPose(flippedTargetPose)
                        .getX()) > FieldConstants.MAX_PLAYER_STATION_X_ZONE
                    || Math.abs(driveBase.getDistanceToPose(flippedTargetPose)
                        .getY()) > FieldConstants.MAX_PLAYER_STATION_Y_ZONE);
  }

  /**
   * Returns a command to follow a path out of the Community.
   * 
   * @param crossingPose Where the Stage 1 auton ends
   */
  public Command getStage1AutonPathCommand(Pose2d crossingPose) {
    return new SequentialCommandGroup(
        new TimedCommand(AutonConstants.DRIVE_BACK_TIME,
            new DriveBaseStraightCommand(driveBase, AutonConstants.DRIVE_BACK_SPEED, false)),
        new ConstructLaterCommand(() -> getPathToTargetCommand(driveBase, crossingPose)));
    // getGroundPickupCommand(crossingPose)
  }

  /**
   * Returns a command to drive to a ground game piece and pick it up.
   * 
   * @param pickupPose The position to pick up the game piece from
   */
  public Command getGroundPickupCommand(Pose2d pickupPose) {
    return new SequentialCommandGroup(new ArmToPoseWithRetractionCommand(arm, ArmPresets.GROUND_PICKUP, 1),
        new ParallelRaceGroup(
            new ArmTowardsPoseCommand(arm, ArmPresets.GROUND_PICKUP),
            new ConstructLaterCommand(
                () -> new DriveBasePathFollowCommand(driveBase,
                    driveBase.generatePath(
                        driveBase.flipWaypointBasedOnAlliance(pickupPose, true)))
                            .andThen(new WaitCommand(0.5))),
            new IntakeSpinCommand(intake, IntakeConstants.INTAKE_VOLTAGE).asProxy()),
        new ArmToPoseWithRetractionCommand(arm, ArmPresets.STOWED, 1));
  }

  /**
   * Returns a command to drive back into the Community and score a game piece
   * 
   * @param crossingInPose Where the robot re-enters the Community
   * @param scoringPosition Which column the robot scores its game piece at
   */
  public Command getStage2AutonCommand(Pose2d crossingInPose, int scoringPosition) {
    Pose2d scoringPose = driveBase.flipWaypointBasedOnAlliance(FieldConstants.SCORING_WAYPOINTS[scoringPosition], true);
    return new SequentialCommandGroup(
        new DriveBaseTurnToAngleCommand(driveBase, scoringPose.getRotation(), 1),
        new ConstructLaterCommand(
            () -> new DriveBasePathFollowCommand(driveBase, driveBase.generatePath(crossingInPose))),
        new ConstructLaterCommand(() -> getPathToTargetCommand(driveBase, scoringPose)));

  }

  public Command getPathToTargetCommand(DriveBase driveBase, Pose2d targetPose) {
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
      index = MathUtil.clamp(index, 0, FieldConstants.TRAVELING_WAYPOINTS.length - 1);
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
      if (index >= FieldConstants.TRAVELING_WAYPOINTS.length - 1) {
        int secondToLast = FieldConstants.TRAVELING_WAYPOINTS.length - 1;
        waypoints.add(
            driveBase.flipWaypointBasedOnAlliance(new Pose2d(FieldConstants.TRAVELING_WAYPOINTS[secondToLast].getX(),
                FieldConstants.TRAVELING_WAYPOINTS[secondToLast].getY(), Rotation2d.fromDegrees(90)), false));
      } else {
        return new DriveBasePathFollowCommand(driveBase, driveBase.generatePath(targetPose));
      }
    }

    return new DriveBasePathFollowCommand(driveBase, driveBase.generatePath(waypoints))
        .andThen(
            new DriveBaseTurnToAngleCommand(driveBase, targetPose.getRotation(), PathConstants.TURN_ANGLE_DEADBAND))
        // .andThen(new WaitCommand(0.25))
        .andThen(
            new ConstructLaterCommand(
                () -> new DriveBasePathFollowCommand(driveBase, driveBase.generatePath(false, 1.5, 1, targetPose))));
  }

}
