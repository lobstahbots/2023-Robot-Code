// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTablesJNI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.ScoringPositionConstants;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.ScoringSystemConstants.ArmConstants;
import frc.robot.Constants.ScoringSystemConstants.ElevatorConstants;
import frc.robot.Constants.ScoringSystemConstants.IntakeConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.OIConstants.DriverConstants;
import frc.robot.Constants.OIConstants.OperatorConstants;
import frc.robot.auton.AutonGenerator;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.scoring.ScoringSystemTowardsPositionCommand;
import frc.robot.commands.scoring.ScoringSystemTowardsPositionWithRetractionCommand;
import frc.robot.commands.scoring.elevator.ResetElevatorCommand;
import frc.robot.commands.scoring.intake.SpinIntakeCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
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

  private final Arm arm = new Arm(ArmConstants.LEFT_MOTOR_ID, ArmConstants.RIGHT_MOTOR_ID,
      ArmConstants.ENCODER_CHANNEL);
  private final Elevator elevator =
      new Elevator(ElevatorConstants.ELEVATOR_MOTOR_ID, ElevatorConstants.ENCODER_CHANNEL_A,
          ElevatorConstants.ENCODER_CHANNEL_B, ElevatorConstants.LIMIT_SWITCH_CHANNEL);
  private final Intake intake = new Intake(IntakeConstants.LEFT_MOTOR_ID, IntakeConstants.RIGHT_MOTOR_ID);

  private final AutonGenerator autonGenerator = new AutonGenerator(driveBase, arm, elevator, intake);

  private final LobstahGamepad driverJoystick = new LobstahGamepad(DriverConstants.DRIVER_JOYSTICK_INDEX);
  private final JoystickButton slowdownButton = driverJoystick.button(DriverConstants.SLOWDOWN_BUTTON_INDEX);

  private final LobstahGamepad operatorJoystick = new LobstahGamepad(OperatorConstants.OPERATOR_JOYSTICK_INDEX);
  private final JoystickButton intakeButton = operatorJoystick.button(OperatorConstants.INTAKE_BUTTON_INDEX);
  private final JoystickButton outtakeButton = operatorJoystick.button(OperatorConstants.OUTTAKE_BUTTON_INDEX);
  private final JoystickButton manualControlButton =
      operatorJoystick.button(OperatorConstants.MANUAL_CONTROL_BUTTON_INDEX);
  private final JoystickButton lowGoalButton = operatorJoystick.button(OperatorConstants.LOW_GOAL_BTN_INDEX);
  private final JoystickButton midGoalButton = operatorJoystick.button(OperatorConstants.MID_GOAL_BTN_INDEX);
  private final JoystickButton highGoalButton = operatorJoystick.button(OperatorConstants.HIGH_GOAL_BTN_INDEX);
  private final POVButton playerStationButton =
      new POVButton(operatorJoystick, OperatorConstants.STATION_PICKUP_POV_INDEX);
  private final POVButton placePieceButton = new POVButton(operatorJoystick, OperatorConstants.PLACE_CONE_POV_INDEX);
  private double lastRecordedTime = 0;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSmartDash();
    configureButtonBindings();
    PathPlannerServer.startServer(5811);
  }

  public double getJoystickLatency() {
    double latency = NetworkTablesJNI.now() - lastRecordedTime;
    lastRecordedTime = NetworkTablesJNI.now();
    SmartDashboard.putNumber("Latency", latency);
    return 1;
  }

  public ScoringPosition getArmPosition() {
    return ScoringPosition.fromArmElevator(arm.getRotation(),
        elevator.getExtension());
  }

  public boolean insideBumpers() {
    return ScoringPosition
        .fromArmElevator(arm.getRotation(), elevator.getExtension())
        .isInsideBumperZone();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    intakeButton.whileTrue(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE));
    outtakeButton.whileTrue(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE));
    manualControlButton.whileTrue(new ScoringSystemTowardsPositionCommand(arm, elevator,
        () -> ScoringPosition.fromArmElevator(Rotation2d.fromDegrees(arm.getSetpoint()), elevator.getSetpointExtension())
            .translateBy(new Translation2d(
                -operatorJoystick.getRawAxis(OperatorConstants.HORIZONTAL_ARM_MOVEMENT_AXIS) * getJoystickLatency()
                    * OperatorConstants.MANUAL_CONTROL_SPEED,
                -operatorJoystick.getRawAxis(OperatorConstants.VERTICAL_ARM_MOVEMENT_AXIS) * getJoystickLatency()
                    * OperatorConstants.MANUAL_CONTROL_SPEED))));
    highGoalButton
        .whileTrue(new ScoringSystemTowardsPositionWithRetractionCommand(arm, elevator,
            ScoringPositionConstants.HIGH_GOAL_SCORING));
    midGoalButton
        .whileTrue(new ScoringSystemTowardsPositionWithRetractionCommand(arm, elevator,
            ScoringPositionConstants.MID_GOAL_SCORING));
    lowGoalButton.whileTrue(new ScoringSystemTowardsPositionWithRetractionCommand(arm, elevator,
        ScoringPositionConstants.LOW_GOAL_SCORING));
    playerStationButton
        .whileTrue(new ScoringSystemTowardsPositionWithRetractionCommand(arm, elevator,
            ScoringPositionConstants.PLAYER_STATION_PICKUP));

    slowdownButton.whileTrue(new TankDriveCommand(driveBase,
        () -> DriverConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverConstants.LEFT_AXIS),
        () -> DriverConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverConstants.RIGHT_AXIS),
        DriverConstants.SQUARED_INPUTS));
  }

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final SendableChooser<Integer> initialPosition = new SendableChooser<>();
  private final SendableChooser<Integer> crossingPosition = new SendableChooser<>();
  private final SendableChooser<Integer> endingPosition = new SendableChooser<>();
  private final SendableChooser<Integer> targetPosition = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  public void configureSmartDash() {
    initialPosition.addOption("Furthest Down", 0);
    initialPosition.addOption("Middle", 1);
    initialPosition.addOption("Furthest Up", 2);
    initialPosition.setDefaultOption("Furthest Up", 1);
    crossingPosition.addOption("Below Platform", 0);
    crossingPosition.addOption("Middle", 1);
    crossingPosition.addOption("Above Platform", 2);
    crossingPosition.setDefaultOption("Middle", 1);
    endingPosition.addOption("Towards Player Station", 3);
    endingPosition.addOption("Slightly Up", 2);
    endingPosition.addOption("Slightly Down", 1);
    endingPosition.addOption("Bottom Corner", 0);
    endingPosition.setDefaultOption("Slightly Up", 2);
    autonChooser.addOption("Path Follow Auton",
        autonGenerator.getPathFollowCommand(initialPosition.getSelected(), crossingPosition.getSelected(),
            endingPosition.getSelected()));
    autonChooser.addOption("Simple Auton", autonGenerator.getSimpleAutonCommand());
    autonChooser.addOption("Do Nothing Auton", new StopDriveCommand(driveBase));
    autonChooser.addOption("Place Piece on Mid Goal Auton",
        autonGenerator.getScoreCommand(ScoringPositionConstants.MID_GOAL_SCORING));
    autonChooser.addOption("Place Piece on High Goal Auton",
        autonGenerator.getScoreCommand(ScoringPositionConstants.HIGH_GOAL_SCORING));
    targetPosition.addOption("0", 0);
    targetPosition.addOption("1", 1);
    targetPosition.addOption("2", 2);
    targetPosition.addOption("3", 3);
    targetPosition.addOption("4", 4);
    targetPosition.addOption("5", 5);
    targetPosition.addOption("6", 6);
    targetPosition.addOption("7", 7);
    targetPosition.addOption("8", 8);
    targetPosition.setDefaultOption("0", 0);
    SmartDashboard.putData("Auton Chooser", autonChooser);
    SmartDashboard.putData("Initial Position Chooser", initialPosition);
    SmartDashboard.putData("Crossing Position Chooser", crossingPosition);
    SmartDashboard.putData("Ending Position Chooser", endingPosition);
    SmartDashboard.putData("Teleop Target", targetPosition);
  }

  /**
   * Updates the robot target for teleop with input from Shuffleboard.
   */
  public Pose2d updateTarget() {
    return FieldConstants.SCORING_WAYPOINTS[targetPosition.getSelected()];
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
        new ResetElevatorCommand(elevator),
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
        new ScoringSystemTowardsPositionWithRetractionCommand(arm, elevator,
            ScoringPositionConstants.STOWED));
    intake.setDefaultCommand(new SpinIntakeCommand(intake, IntakeConstants.PASSIVE_INTAKE_VOLTAGE));
  }

  /**
   * Robot.java should run this method when auton starts. This method should be used to set the default commands for
   * subsystems while in auton. If you set a default here, set a corresponding teleop default in
   * setTeleopDefaultCommands().
   */
  public void setAutonDefaultCommands() {
    arm.setIdleMode(IdleMode.kBrake);
    elevator.setIdleMode(IdleMode.kBrake);
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }

  /**
   * Robot.java should run this method when test mode starts. This method should be used to set the default commands for
   * subsystems while in test mode.
   */
  public void setTestDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
    arm.setIdleMode(IdleMode.kCoast);
    elevator.setIdleMode(IdleMode.kCoast);
  }

  /**
   * Robot.java should run this method when robot simulation starts. This method should be used to set the default
   * commands for subsystems while running a simulation.
   */
  public void setSimDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }
}
