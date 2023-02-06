// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.UIConstants;
import frc.robot.Constants.UIConstants.DriverAxes;
import frc.robot.auton.AutonGenerator;
import frc.robot.commands.drive.TargetCommand;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subsystems.DriveBase;
import lobstah.stl.io.LobstahGamepad;

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

  private final AutonGenerator autonGenerator = new AutonGenerator(driveBase);

  private final LobstahGamepad driverJoystick = new LobstahGamepad(UIConstants.DRIVER_JOYSTICK_INDEX);
  private final LobstahGamepad operatorJoystick = new LobstahGamepad(UIConstants.OPERATOR_JOYSTICK_INDEX);

  private final JoystickButton button = driverJoystick.button(1);
  private final JoystickButton slowdownButton = driverJoystick.button(UIConstants.SLOWDOWN_BUTTON_INDEX);

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
    button.whileTrue(new TargetCommand(driveBase, () -> (this.updateTarget())));
    slowdownButton.whileTrue(new TankDriveCommand(driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT * driverJoystick.getRawAxis(DriverAxes.RIGHT),
        UIConstants.SQUARED_INPUTS));
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
    autonChooser.addOption("Do Nothing Auton", null);
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
    return autonChooser.getSelected();
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
            () -> -driverJoystick.getRawAxis(DriverAxes.LEFT),
            () -> -driverJoystick.getRawAxis(DriverAxes.RIGHT),
            UIConstants.SQUARED_INPUTS));
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
