// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.Constants.ArmPresets;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants.DriverConstants;
import frc.robot.Constants.OIConstants.OperatorConstants;
import frc.robot.auton.AutonGenerator;
import frc.robot.commands.arm.ArmToPoseCommand;
import frc.robot.commands.arm.ArmToPoseWithRetractionCommand;
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

    // Target selection
    new POVButton(operatorJoystick, OperatorConstants.SHIFT_SELECTION_LEFT_POV)
        .onTrue(new InstantCommand(() -> targetSelector.changeColumn(-1)));
    new POVButton(operatorJoystick, OperatorConstants.SHIFT_SELECTION_RIGHT_POV)
        .onTrue(new InstantCommand(() -> targetSelector.changeColumn(1)));
    new POVButton(operatorJoystick, OperatorConstants.SHIFT_SELECTION_UP_POV)
        .onTrue(new InstantCommand(() -> targetSelector.changeRow(-1)));
    new POVButton(operatorJoystick, OperatorConstants.SHIFT_SELECTION_DOWN_POV)
        .onTrue(new InstantCommand(() -> targetSelector.changeRow(1)));

    // Scoring
    JoystickButton scoreLineupButton = operatorJoystick.button(OperatorConstants.SCORE_LINEUP_BTN);
    scoreLineupButton.whileTrue(
        new InstantCommand(/* TODO: Line up */).asProxy()
            .andThen(new ArmToPoseWithRetractionCommand(arm, null /* TODO */, 5).asProxy())
            .andThen(new ArmTowardsPoseCommand(arm, () -> arm.getSetpointPose()).asProxy().repeatedly()));
    operatorJoystick.button(OperatorConstants.SCORE_PLACE_BTN).and(scoreLineupButton).onTrue(
        new ArmToPoseCommand(arm, () -> arm.getSetpointPose().translateBy(ArmPresets.CONE_SCORING_DROPDOWN), 2)
            .andThen(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE)
                .alongWith(Commands.waitSeconds(1).andThen(new ArmToPoseCommand(arm,
                    () -> arm.getSetpointPose().translateBy(ArmPresets.CONE_SCORING_DROPDOWN.unaryMinus()), 5))))
            .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));

    // Manual adjustment
    operatorJoystick.button(OperatorConstants.MANUAL_ADJUSTMENT_BTN).whileTrue(new ArmTowardsPoseCommand(arm,
        () -> arm.getSetpointPose().translateBy(new Translation2d(
            MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorConstants.X_ADJUSTMENT_JOYSTICK_AXIS),
                OperatorConstants.JOYSTICK_DEADBAND) * OperatorConstants.MANUAL_CONTROL_SPEED,
            MathUtil.applyDeadband(-operatorJoystick.getRawAxis(OperatorConstants.Y_ADJUSTMENT_JOYSTICK_AXIS),
                OperatorConstants.JOYSTICK_DEADBAND) * OperatorConstants.MANUAL_CONTROL_SPEED)))
                    .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));


    // Pickup
    operatorJoystick.button(OperatorConstants.GROUND_PICKUP_BTN)
        .whileTrue(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE)
            .alongWith(new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.GROUND_PICKUP)));
    operatorJoystick.button(OperatorConstants.LEFT_PICKUP_BTN)
        .whileTrue(new InstantCommand(/* TODO: Line up */).asProxy()
            .andThen(new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.PLAYER_STATION_PICKUP)
                .alongWith(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE))));
    operatorJoystick.button(OperatorConstants.LEFT_PICKUP_BTN)
        .whileTrue(new InstantCommand(/* TODO: Line up */).asProxy()
            .andThen(new ArmTowardsPoseWithRetractionCommand(arm, ArmPresets.PLAYER_STATION_PICKUP)
                .alongWith(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE))));

    // Legacy operator controls (TODO)
  }

  public Pose2d getScoreColumn() {
    return FieldConstants.SCORING_WAYPOINTS[targetSelector.getColumn()];
  }

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final SendableChooser<Integer> initialPosition = new SendableChooser<>();
  private final SendableChooser<Integer> crossingPosition = new SendableChooser<>();
  private final SendableChooser<Integer> endingPosition = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  public void configureSmartDash() {
    initialPosition.addOption("0", 0);
    initialPosition.addOption("1", 1);
    initialPosition.addOption("2", 2);
    initialPosition.addOption("3", 3);
    initialPosition.addOption("4", 4);
    initialPosition.addOption("5", 5);
    initialPosition.addOption("6", 6);
    initialPosition.addOption("7", 7);
    initialPosition.addOption("8", 8);
    initialPosition.setDefaultOption("0", 0);
    crossingPosition.addOption("Right of Platform", 0);
    crossingPosition.addOption("Left of Platform", 1);
    crossingPosition.setDefaultOption("Left of Platform", 1);
    endingPosition.addOption("Towards Player Station", 3);
    endingPosition.addOption("Slightly Left", 2);
    endingPosition.addOption("Slightly Right", 1);
    endingPosition.addOption("Right Side", 0);
    endingPosition.setDefaultOption("Slightly Left", 2);
    autonChooser.addOption("Path Follow Auton",
        autonGenerator.getPathFollowCommand(initialPosition.getSelected(), crossingPosition.getSelected(),
            endingPosition.getSelected()));
    autonChooser.addOption("Simple Auton", autonGenerator.getSimpleAutonCommand());
    autonChooser.addOption("Do Nothing Auton", new StopDriveCommand(driveBase));
    autonChooser.addOption("Place Piece on Mid Goal Auton",
        autonGenerator.getScoreCommand(ArmPresets.MID_GOAL_SCORING));
    autonChooser.addOption("Place Piece on High Goal Auton",
        autonGenerator.getScoreCommand(ArmPresets.HIGH_GOAL_SCORING));
    SmartDashboard.putData("Auton Chooser", autonChooser);
    SmartDashboard.putData("Initial Position Chooser", initialPosition);
    SmartDashboard.putData("Crossing Position Chooser", crossingPosition);
    SmartDashboard.putData("Ending Position Chooser", endingPosition);
    SmartDashboard.putData("Teleop Target Selector", targetSelector);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new ResetElevatorCommand(arm),
        autonChooser.getSelected());
  }

  /**
   * Robot.java should run this method when teleop starts. This method should be used to set the default commands for
   * subsystems while in teleop. If you set a default here, set a corresponding auton default in
   * setAutonDefaultCommands().
   */
  public void setTeleopDefaultCommands() {
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
    driveBase.initGyro();
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
