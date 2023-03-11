// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.InternalButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.ArmPresets;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants.DriverConstants;
import frc.robot.Constants.OIConstants.OperatorConstants;
import frc.robot.auton.AutonChooser;
import frc.robot.auton.AutonGenerator;
import frc.robot.commands.arm.ArmToPoseCommand;
import frc.robot.commands.arm.ArmTowardsPoseCommand;
import frc.robot.commands.arm.ArmTowardsPoseWithRetractionCommand;
import frc.robot.commands.arm.elevator.ResetElevatorCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.intake.SpinIntakeCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.arm.Elevator;
import frc.robot.subsystems.arm.Pivot;
import lobstah.stl.command.ConstructLaterCommand;
import lobstah.stl.oi.LobstahGamepad;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final DriveBase driveBase = new DriveBase(
      DriveMotorCANIDs.LEFT_FRONT,
      DriveMotorCANIDs.LEFT_BACK,
      DriveMotorCANIDs.RIGHT_FRONT,
      DriveMotorCANIDs.RIGHT_BACK);

  private final Arm arm = new Arm(
      new Pivot(PivotConstants.LEFT_MOTOR_ID, PivotConstants.RIGHT_MOTOR_ID,
          PivotConstants.ENCODER_CHANNEL),
      new Elevator(ElevatorConstants.ELEVATOR_MOTOR_ID, ElevatorConstants.ENCODER_CHANNEL_A,
          ElevatorConstants.ENCODER_CHANNEL_B, ElevatorConstants.LIMIT_SWITCH_CHANNEL));

  private final Intake intake =
      new Intake(Constants.IntakeConstants.LEFT_MOTOR_ID, Constants.IntakeConstants.RIGHT_MOTOR_ID);

  private final AutonGenerator autonGenerator = new AutonGenerator(driveBase, arm, intake);
  private final TargetSelector targetSelector = new TargetSelector();

  private final LobstahGamepad driverJoystick = new LobstahGamepad(DriverConstants.DRIVER_USB_INDEX);
  private final LobstahGamepad operatorJoystick = new LobstahGamepad(OperatorConstants.OPERATOR_USB_INDEX);

  private ArmPose manualTarget;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSmartDash();
    configureButtonBindings();
    PathPlannerServer.startServer(5811);
  }

  /**
   * Zeroes the gyro.
   */
  public void initGyro() {
    driveBase.zeroGyro();
  }

  /**
   * @return Whether the robot is within the scoring zone.
   */
  public boolean canDriveToTarget() {
    if (DriverStation.getAlliance() == Alliance.Red) {
      return driveBase.getPose().getX() >= FieldConstants.FIELD_LENGTH - FieldConstants.SCORING_ZONE_X;
    } else {
      return driveBase.getPose().getX() <= FieldConstants.SCORING_ZONE_X;
    }
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    // Drive slowdown
    driverJoystick.button(DriverConstants.SLOWDOWN_BTN).whileTrue(new TankDriveCommand(driveBase,
        () -> DriverConstants.SLOWDOWN_FACTOR * driverJoystick.getRawAxis(DriverConstants.LEFT_AXIS),
        () -> DriverConstants.SLOWDOWN_FACTOR * driverJoystick.getRawAxis(DriverConstants.RIGHT_AXIS),
        DriverConstants.SQUARED_INPUTS));

    // operatorJoystick.button(OperatorConstants.PLACE_DOWN_BTN).onTrue(
    // new ArmToPoseCommand(arm, () -> arm.getSetpointPose().translateBy(ArmPresets.CONE_SCORING_DROPDOWN), 2)
    // .andThen(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE)
    // .alongWith(Commands.waitSeconds(1).andThen(new ArmToPoseCommand(arm,
    // () -> arm.getSetpointPose().translateBy(ArmPresets.CONE_SCORING_DROPDOWN.unaryMinus()), 2)))));

    // Manual adjustment
    // TODO: Shouldn't need to cache manual control target
    operatorJoystick.button(OperatorConstants.MANUAL_CONTROL_BTN)
        .whileTrue(new InstantCommand(() -> manualTarget = arm.getSetpointPose()).andThen(new ArmTowardsPoseCommand(arm,
            () -> {
              if (manualTarget.isInsideStowedZone()) {
                manualTarget = ArmPose.fromAngleExtension(manualTarget.getAngle().plus(
                    Rotation2d.fromDegrees(MathUtil
                        .applyDeadband(-operatorJoystick.getRawAxis(OperatorConstants.MANUAL_Y_JOYSTICK_AXIS),
                            OperatorConstants.JOYSTICK_DEADBAND)
                        * OperatorConstants.MANUAL_CONTROL_SPEED)),
                    0);
              }
              if (manualTarget.getY() < ArmConstants.MIN_Y_POSITION) {
                manualTarget = manualTarget.translateBy(new Translation2d(
                    MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorConstants.MANUAL_X_JOYSTICK_AXIS),
                        OperatorConstants.JOYSTICK_DEADBAND) * OperatorConstants.MANUAL_CONTROL_SPEED,
                    MathUtil.clamp(MathUtil.applyDeadband(
                        -operatorJoystick.getRawAxis(OperatorConstants.MANUAL_Y_JOYSTICK_AXIS),
                        OperatorConstants.JOYSTICK_DEADBAND) * OperatorConstants.MANUAL_CONTROL_SPEED, 0, 10)));
              }
              manualTarget = manualTarget.translateBy(new Translation2d(
                  MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorConstants.MANUAL_X_JOYSTICK_AXIS),
                      OperatorConstants.JOYSTICK_DEADBAND) * OperatorConstants.MANUAL_CONTROL_SPEED,
                  MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorConstants.MANUAL_Y_JOYSTICK_AXIS),
                      OperatorConstants.JOYSTICK_DEADBAND) * OperatorConstants.MANUAL_CONTROL_SPEED));
              return manualTarget;
            })
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming)));


    // Pickup
    operatorJoystick.button(OperatorConstants.GROUND_PICKUP_BTN)
        .whileTrue(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE)
            .alongWith(new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.GROUND_PICKUP)));

    // Legacy operator controls

    operatorJoystick.pov(OperatorConstants.LOW_GOAL_POV)
        .whileTrue(new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.GROUND_PICKUP));
    operatorJoystick.pov(OperatorConstants.MID_GOAL_POV)
        .whileTrue(new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.MID_GOAL_SCORING));
    operatorJoystick.pov(OperatorConstants.HIGH_GOAL_POV)
        .whileTrue(new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.HIGH_GOAL_SCORING));

    operatorJoystick.button(OperatorConstants.INTAKE_BTN)
        .whileTrue(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE));
    operatorJoystick.button(OperatorConstants.OUTTAKE_BTN)
        .whileTrue(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE));
  }

  /**
   * Configures "left" and "right" Player station buttons depending on alliance color.
   */
  public void configurePlayerStationButtons() {
    if (DriverStation.getAlliance() == Alliance.Blue) {
      operatorJoystick.button(OperatorConstants.LEFT_PICKUP_BTN)
          .whileTrue(autonGenerator.getDriveToPlayerStationCommand(FieldConstants.PLAYER_STATION_PICKUP_LEFT));
      operatorJoystick.button(OperatorConstants.RIGHT_PICKUP_BTN)
          .whileTrue(autonGenerator.getDriveToPlayerStationCommand(FieldConstants.PLAYER_STATION_PICKUP_RIGHT));
    } else {
      operatorJoystick.button(OperatorConstants.LEFT_PICKUP_BTN)
          .whileTrue(autonGenerator.getDriveToPlayerStationCommand(FieldConstants.PLAYER_STATION_PICKUP_RIGHT));
      operatorJoystick.button(OperatorConstants.RIGHT_PICKUP_BTN)
          .whileTrue(autonGenerator.getDriveToPlayerStationCommand(FieldConstants.PLAYER_STATION_PICKUP_LEFT));
    }
  }

  /**
   * @return The target pose based on the selected color alliance.
   */
  public Pose2d getScoreColumn() {
    return driveBase.flipWaypointBasedOnAlliance(
        FieldConstants.SCORING_WAYPOINTS[flipColumnBasedOnAlliance(targetSelector.getColumn())], true);
  }

  /**
   * Flips the selected column based on alliance color.
   */

  public int flipColumnBasedOnAlliance(int column) {
    if (column > 8 || column < 0) {
      return 0;
    }
    if (DriverStation.getAlliance() == Alliance.Red) {
      return 8 - column;
    } else {
      return column;
    }
  }

  private final AutonChooser autonChooser = new AutonChooser();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  public void configureSmartDash() {
    SmartDashboard.putData("Auton Chooser", autonChooser);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    Command selected = null;
    switch (autonChooser.getRoutine()) {
      case "Line":
        selected = autonGenerator.getPathFollowCommand(autonChooser.getStartingColumn(), autonChooser.getCrossingSide(),
            0);
      case "Score":
        selected = autonGenerator.getScoreCommand(autonChooser.getRow());
        break;
      case "Score + Line":
        selected = autonGenerator.getScoreAndDriveCommand(autonChooser.getRow(), autonChooser.getStartingColumn(),
            autonChooser.getCrossingSide(),
            0);
        break;
      default:
        DriverStation.reportError("Invalid autonomous routine selected!", true);
      case "None":
        selected = new StopDriveCommand(driveBase);

    }
    return new SequentialCommandGroup(
        new ResetElevatorCommand(arm).asProxy(),
        selected);
  }

  public void initOdometry() {
    driveBase.initOdometry(
        driveBase.flipWaypointBasedOnAlliance(
            FieldConstants.SCORING_WAYPOINTS[flipColumnBasedOnAlliance(autonChooser.getStartingColumn())], true));
  }

  /**
   * Robot.java should run this method when teleop starts. This method should be used to set the default commands for
   * subsystems while in teleop. If you set a default here, set a corresponding auton default in
   * setAutonDefaultCommands().
   */
  public void setTeleopDefaultCommands() {
    CommandScheduler.getInstance().schedule(new ResetElevatorCommand(arm));
    driveBase.setNeutralMode(NeutralMode.Brake);
    arm.getPivot().setIdleMode(IdleMode.kBrake);
    arm.getElevator().setIdleMode(IdleMode.kBrake);
    driveBase.setDefaultCommand(
        new TankDriveCommand(
            driveBase,
            () -> -driverJoystick.getRawAxis(DriverConstants.LEFT_AXIS),
            () -> -driverJoystick.getRawAxis(DriverConstants.RIGHT_AXIS),
            DriverConstants.SQUARED_INPUTS));
    arm.setDefaultCommand(
        new ArmTowardsPoseWithRetractionCommand(arm,
            ArmPresets.STOWED));
    intake.setDefaultCommand(new SpinIntakeCommand(intake, Constants.IntakeConstants.PASSIVE_INTAKE_VOLTAGE));
  }

  /**
   * Robot.java should run this method when auton starts. This method should be used to set the default commands for
   * subsystems while in auton. If you set a default here, set a corresponding teleop default in
   * setTeleopDefaultCommands().
   */
  public void setAutonDefaultCommands() {
    driveBase.setNeutralMode(NeutralMode.Brake);
    arm.getPivot().setIdleMode(IdleMode.kBrake);
    arm.getElevator().setIdleMode(IdleMode.kBrake);
    intake.setDefaultCommand(new SpinIntakeCommand(intake, Constants.IntakeConstants.PASSIVE_INTAKE_VOLTAGE));
    arm.setDefaultCommand(
        new ArmTowardsPoseWithRetractionCommand(arm,
            ArmPresets.STOWED));
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }

  /**
   * Robot.java should run this method when test mode starts. This method should be used to set the default commands for
   * subsystems while in test mode.
   */
  public void setTestDefaultCommands() {
    driveBase.setNeutralMode(NeutralMode.Coast);
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
    arm.getPivot().setIdleMode(IdleMode.kCoast);
    arm.getElevator().setIdleMode(IdleMode.kCoast);
  }

  /**
   * Robot.java should run this method when robot simulation starts. This method should be used to set the default
   * commands for subsystems while running a simulation.
   */
  public void setSimDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }
}
