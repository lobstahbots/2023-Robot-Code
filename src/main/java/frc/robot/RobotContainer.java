// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutonConstants;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriveConstants.DriveMotorCANIDs;
import frc.robot.Constants.IOConstants;
import frc.robot.Constants.IOConstants.DriverAxes;
import frc.robot.Constants.IOConstants.DriverButtons;

import frc.robot.commands.drive.StopDriveCommand;
import frc.robot.commands.drive.StraightDriveCommand;
import frc.robot.commands.drive.TankDriveCommand;
import frc.robot.subsystems.DriveBase;
import lobstahbots.stl.command.TimedCommand;
import lobstahbots.stl.io.LobstahBotsController;

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

  private final LobstahBotsController driverJoystick = new LobstahBotsController(IOConstants.DRIVER_JOYSTICK_INDEX);

  private final JoystickButton slowdownButton1 = driverJoystick.button(DriverButtons.SLOWDOWN1);
  private final JoystickButton slowdownButton2 = driverJoystick.button(DriverButtons.SLOWDOWN2);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();
    configureSmartDash();
  }

  /**
   * Use this method to define your button->command mappings.
   */
  private void configureButtonBindings() {

    slowdownButton1.whileHeld(new TankDriveCommand(
        driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT1 * -driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT1 * -driverJoystick.getRawAxis(DriverAxes.RIGHT),
        true));

    slowdownButton2.whileHeld(new TankDriveCommand(
        driveBase,
        () -> DriveConstants.SLOWDOWN_PERCENT2 * -driverJoystick.getRawAxis(DriverAxes.LEFT),
        () -> DriveConstants.SLOWDOWN_PERCENT2 * -driverJoystick.getRawAxis(DriverAxes.RIGHT),
        true));

  }

  // A simple auto routine that drives in a straight line.
  private final Command driveAuton =
      new TimedCommand(
          AutonConstants.SIMPLE_AUTON_RUNTIME,
          new StraightDriveCommand(
              driveBase,
              AutonConstants.SIMPLE_AUTON_SPEED, false));

  // An auto routine that does nothing.
  private final Command doNothingAuton = null;

  private final SendableChooser<Command> autonChooser = new SendableChooser<>();

  /**
   * Use this method to run tasks that configure sendables and other smartdashboard items.
   */
  private void configureSmartDash() {
    autonChooser.addOption("Drive Auton", driveAuton);
    autonChooser.addOption("Do Nothing Auton", doNothingAuton);

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
            true));
  }

  /**
   * Robot.java should run this method when auton starts. This method should be used to set the default commands for
   * subsystems while in auton. If you set a default here, set a corresponding teleop default in
   * setTeleopDefaultCommands().
   */
  public void setAutonDefaultCommands() {
    driveBase.setDefaultCommand(new StopDriveCommand(driveBase));
  }


}
