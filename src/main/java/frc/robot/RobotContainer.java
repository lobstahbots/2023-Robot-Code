// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IOConstants.DriverAxes;
import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.AutonGenerator;
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

  private final LobstahGamepad driverJoystick = new LobstahGamepad(IOConstants.DRIVER_JOYSTICK_INDEX);
  private final LobstahGamepad operatorJoystick = new LobstahGamepad(IOConstants.OPERATOR_JOYSTICK_INDEX);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    configureSmartDash();
    PathPlannerServer.startServer(5811);
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {}

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();
  private final SendableChooser<Integer> initialPosition = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  private void configureSmartDash() {
    autonChooser.addOption("Simple Auton", autonGenerator.getSimpleAutonCommand());
    autonChooser.addOption("Do Nothing Auton", autonGenerator.getDoNothingCommand());
    initialPosition.addOption("Furthest Left", 0);
    initialPosition.setDefaultOption("Furthest Left", 0);
    autonChooser.addOption("Path Follow Auton",
        autonGenerator.getPathFollowCommand(initialPosition.getSelected(), 1, 1));
    SmartDashboard.putData(autonChooser);
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
            IOConstants.SQUARED_INPUTS));
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
