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

/**
 * Makes an {@link DriveBase} follow a given PathPlannerTrajectory using a Ramsete Controller.
 */
public class PathFollowCommand extends PPRamseteCommand {


  /**
   * Drives an {@link DriveBase} through the provided PathPlannerTrajectory using a Ramsete Controller.
   */
  public PathFollowCommand(DriveBase driveBase, PathPlannerTrajectory traj) {
    super(
        traj,
        driveBase::getPose,
        new RamseteController(),
        new SimpleMotorFeedforward(PathConstants.kS, PathConstants.kV, PathConstants.kA),
        DriveConstants.KINEMATICS,
        driveBase::getWheelSpeeds,
        new PIDController(PathConstants.kP, PathConstants.kI, PathConstants.KD),
        new PIDController(PathConstants.kP, PathConstants.kI, PathConstants.KD),
        driveBase::tankDriveVoltage,
        true,
        driveBase);
  }
}
