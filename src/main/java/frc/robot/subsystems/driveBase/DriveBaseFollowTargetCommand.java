// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.driveBase;

import java.util.Optional;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.photonvision.PhotonVision;
import lobstah.stl.math.LobstahMath;
import org.photonvision.targeting.PhotonTrackedTarget;

public class DriveBaseFollowTargetCommand extends CommandBase {
  private final DriveBase driveBase;
  private final PhotonVision photonVision;

  /** Creates a new DriveBaseFollowTargetCommand. */
  public DriveBaseFollowTargetCommand(DriveBase driveBase, PhotonVision photonVision) {
    this.driveBase = driveBase;
    this.photonVision = photonVision;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Optional<PhotonTrackedTarget> target = photonVision.getLowestAmbiguityTarget(photonVision.getTargets());
    if (target.isPresent()) {
      double area = target.get().getArea();
      double speed = 0;
      if (area >= VisionConstants.MAX_TAG_AREA) {
        speed = -VisionConstants.MIN_DRIVE_SPEED;
      } else if (area <= VisionConstants.MIN_TAG_AREA) {
        speed = LobstahMath.scaleNumberToRange(area, VisionConstants.MIN_TAG_AREA, VisionConstants.MAX_TAG_AREA,
            VisionConstants.MIN_DRIVE_SPEED, VisionConstants.MAX_DRIVE_SPEED);
      }

      double turn = LobstahMath.scaleNumberToRange(target.get().getYaw(), VisionConstants.MIN_YAW_ERROR,
          VisionConstants.MAX_YAW_ERROR, VisionConstants.MIN_TURN, VisionConstants.MAX_TURN);

      driveBase.arcadeDrive(speed, turn, false);

    } else {
      driveBase.arcadeDrive(0, 0, false);
    }


  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
