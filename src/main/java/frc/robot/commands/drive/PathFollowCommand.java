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

// NOTE: Consider using this command inline, rather than writing a subclass. For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathFollowCommand extends SequentialCommandGroup {
  /** Creates a new PathCommand. */
  private final RamseteController ramsete;
  private final PIDController leftController;
  private final PIDController rightController;
  private final PPRamseteCommand ramseteCommand;
  private final DriveBase driveBase;

  /** Creates a new PathFollowCommand. */
  public PathFollowCommand(DriveBase drivebase, PathPlannerTrajectory traj, boolean isFirstPath, boolean isLastPath) {
    this.driveBase = drivebase;
    this.ramsete = new RamseteController();
    this.leftController = new PIDController(PathConstants.KP, PathConstants.KI, PathConstants.KD);
    this.rightController = new PIDController(PathConstants.KP, PathConstants.KI, PathConstants.KD);
    ramseteCommand = new PPRamseteCommand(
        traj,
        driveBase::getPose, // Pose supplier
        ramsete,
        new SimpleMotorFeedforward(PathConstants.KS, PathConstants.KV, PathConstants.KA),
        DriveConstants.KINEMATICS, // DifferentialDriveKinematics
        driveBase::getWheelSpeeds, // DifferentialDriveWheelSpeeds supplier
        leftController, // Left controller. Tune these values for your robot. Leaving them 0 will only use
                        // feedforwards.
        rightController, // Right controller (usually the same values as left controller)
        (leftVolts, rightVolts) -> {
          driveBase.tankDriveVolts(leftVolts, rightVolts);
        }, // Voltage biconsumer
        driveBase // Requires this drive subsystem
    );
    addCommands(new InstantCommand(() -> {
      // Reset odometry for the first path you run during auto
      if (isFirstPath) {
        driveBase.resetOdometry(traj.getInitialPose().getTranslation(), traj.getInitialPose().getRotation());
      }
    }), ramseteCommand, new InstantCommand(() -> {
      if (isLastPath) {
        driveBase.stopDrive();
      }
    }));
  }
}
