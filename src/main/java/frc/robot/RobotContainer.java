// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPositionConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.UIConstants.DriverConstants;
import frc.robot.Constants.UIConstants.OperatorConstants;
import frc.robot.auton.AutonGenerator;
import frc.robot.commands.PeriodicConditionalCommand;
import frc.robot.commands.arm.MaintainArmAngleCommand;
import frc.robot.commands.arm.RotateArmCommand;
import frc.robot.commands.arm.RotateArmToAngleCommand;
import frc.robot.commands.arm.MoveArmToPositionCommand;
import frc.robot.commands.arm.elevator.ResetElevatorCommand;
import frc.robot.commands.arm.elevator.RetractElevatorCommand;
import frc.robot.commands.arm.elevator.RunElevatorCommand;
import frc.robot.commands.arm.elevator.RunElevatorToLengthCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.commands.intake.SpinIntakeCommand;
import frc.robot.commands.intake.StopSpinIntakeCommand;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.DriveBase;
import lobstah.stl.io.LobstahGamepad;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

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

  private final AutonGenerator autonGenerator = new AutonGenerator(driveBase);

  private final LobstahGamepad driverJoystick = new LobstahGamepad(DriverConstants.DRIVER_JOYSTICK_INDEX);
  private final LobstahGamepad operatorJoystick = new LobstahGamepad(OperatorConstants.OPERATOR_JOYSTICK_INDEX);

  private final JoystickButton intakeButton = operatorJoystick.button(OperatorConstants.INTAKE_BUTTON_INDEX);
  private final JoystickButton outtakeButton = operatorJoystick.button(OperatorConstants.OUTTAKE_BUTTON_INDEX);
  private final JoystickButton manualControlButton =
      operatorJoystick.button(OperatorConstants.MANUAL_CONTROL_BUTTON_INDEX);
  private final JoystickButton slowdownButton = driverJoystick.button(DriverConstants.SLOWDOWN_BUTTON_INDEX);
  private final JoystickButton lowGoalButton = operatorJoystick.button(OperatorConstants.LOW_GOAL_BTN_INDEX);
  private final JoystickButton midGoalButton = operatorJoystick.button(OperatorConstants.MID_GOAL_BTN_INDEX);
  private final JoystickButton highGoalButton = operatorJoystick.button(OperatorConstants.HIGH_GOAL_BTN_INDEX);
  private final JoystickButton playerStationButton =
      operatorJoystick.button(OperatorConstants.STATION_PICKUP_BTN_INDEX);
  // private final JoystickButton groundPickupButton = operatorJoystick.button(OperatorConstants.LOW_GOAL_BTN_INDEX);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureSmartDash();
    configureButtonBindings();
    PathPlannerServer.startServer(5811);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {
    intakeButton.whileTrue(new SpinIntakeCommand(intake, IntakeConstants.INTAKE_VOLTAGE));
    outtakeButton.whileTrue(new SpinIntakeCommand(intake, IntakeConstants.OUTTAKE_VOLTAGE));
    manualControlButton
        .whileTrue(new ParallelCommandGroup(
            new RunElevatorCommand(elevator, () -> operatorJoystick.getRawAxis(OperatorConstants.ELEVATOR_AXIS)),
            new PeriodicConditionalCommand(
                new RotateArmCommand(arm, () -> operatorJoystick.getRawAxis(OperatorConstants.ARM_AXIS)),
                new MaintainArmAngleCommand(arm),
                () -> Math.abs(operatorJoystick
                    .getRawAxis(OperatorConstants.ARM_AXIS)) > OperatorConstants.JOYSTICK_DEADBAND)));
    slowdownButton.whileTrue(new TankDriveCommand(driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverConstants.LEFT_AXIS),
        () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverConstants.RIGHT_AXIS),
        DriverConstants.SQUARED_INPUTS));

    highGoalButton.whileTrue(new MoveArmToPositionCommand(arm, elevator, ArmPositionConstants.HIGH_GOAL_SCORING));
    midGoalButton.whileTrue(new MoveArmToPositionCommand(arm, elevator, ArmPositionConstants.MID_GOAL_SCORING));
    // groundPickupButton.whileTrue(new MoveArmToPositionCommand(arm, elevator, ArmPositionConstants.GROUND_PICKUP));
    playerStationButton
        .whileTrue(new MoveArmToPositionCommand(arm, elevator, ArmPositionConstants.PLAYER_STATION_PICKUP));
    lowGoalButton.whileTrue(new MoveArmToPositionCommand(arm, elevator, ArmPositionConstants.GROUND_SCORING));
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
    // SmartDashboard.putData("Command Scheduler", CommandScheduler.getInstance());
    // SmartDashboard.putData("Rotate To 50 Command",
    // new RotateArmToAngleCommand(arm, 50));
    // SmartDashboard.putData("Extend To 20 Command", new RunElevatorToLengthCommand(elevator, 20));
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
    elevator
        .setDefaultCommand(new RetractElevatorCommand(elevator));

    // arm.setDefaultCommand(new MaintainArmAngleCommand(arm));
    arm.setDefaultCommand(
        new PeriodicConditionalCommand(new MaintainArmAngleCommand(arm), new RotateArmToAngleCommand(arm, 0),
            () -> elevator.getExtension() > ElevatorConstants.CAN_ROTATE));
    intake.setDefaultCommand(new SpinIntakeCommand(intake, IntakeConstants.PASSIVE_INTAKE_VOLTAGE));
  }

  /**
   * Robot.java should run this method when auton starts. This method should be used to set the default commands for
   * subsystems while in auton. If you set a default here, set a corresponding teleop default in
   * setTeleopDefaultCommands().
   */
  public void setAutonDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }

  /**
   * Robot.java should run this method when test mode starts. This method should be used to set the default commands for
   * subsystems while in test mode.
   */
  public void setTestDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }

  /**
   * Robot.java should run this method when robot simulation starts. This method should be used to set the default
   * commands for subsystems while running a simulation.
   */
  public void setSimDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }
}
