// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Makes an {@link DriveBase} follow a given PathPlannerTrajectory using a Ramsete Controller.
 */
public class PathFollowCommand extends SequentialCommandGroup {
  private final RamseteController ramsete;
  private final PIDController leftController;
  private final PIDController rightController;
  private final PPRamseteCommand ramseteCommand;
  private final DriveBase driveBase;

  /**
   * Drives an {@link DriveBase} through the provided PathPlannerTrajectory using a Ramsete Controller.
   */
  public PathFollowCommand(DriveBase drivebase, PathPlannerTrajectory traj, boolean isFirstPath) {
    this.driveBase = drivebase;
    this.ramsete = new RamseteController();
    this.leftController = new PIDController(PathConstants.KP, PathConstants.KI, PathConstants.KD);
    this.rightController = new PIDController(PathConstants.KP, PathConstants.KI, PathConstants.KD);
    ramseteCommand = new PPRamseteCommand(
        traj,
        driveBase::getPose,
        ramsete,
        new SimpleMotorFeedforward(PathConstants.KS, PathConstants.KV, PathConstants.KA),
        DriveConstants.KINEMATICS,
        driveBase::getWheelSpeeds,
        leftController,
        rightController,
        (leftVolts, rightVolts) -> {
          driveBase.tankDriveVoltage(leftVolts, rightVolts);
        }, driveBase);
    addCommands(new InstantCommand(() -> {
      if (isFirstPath) {
        driveBase.resetOdometry(traj.getInitialPose().getTranslation(), traj.getInitialPose().getRotation());
      }
    }));
    addCommands(ramseteCommand);
  }
}
