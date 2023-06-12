// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driveBase;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;

/**
 * Makes an {@link DriveBase} follow a given PathPlannerTrajectory using a Ramsete Controller.
 */
public class DriveBasePathFollowCommand extends PPRamseteCommand {

  /**
   * Drives an {@link DriveBase} through the provided PathPlannerTrajectory using a Ramsete Controller.
   */
  public DriveBasePathFollowCommand(DriveBase driveBase, PathPlannerTrajectory trajectory) {
    super(
        trajectory,
        driveBase::getPose,
        new RamseteController(PathConstants.RAMSETE_B, PathConstants.RAMSETE_ZETA),
        new SimpleMotorFeedforward(PathConstants.kS, PathConstants.kV, PathConstants.kA),
        DriveConstants.KINEMATICS,
        driveBase::getWheelSpeeds,
        new PIDController(PathConstants.LEFT_kP, PathConstants.kI, PathConstants.KD),
        new PIDController(PathConstants.RIGHT_kP, PathConstants.kI, PathConstants.KD),
        driveBase::tankDriveVoltage,
        false,
        driveBase);
  }
}
