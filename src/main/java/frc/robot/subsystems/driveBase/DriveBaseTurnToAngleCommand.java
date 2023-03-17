
package frc.robot.subsystems.driveBase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PathConstants;

public class DriveBaseTurnToAngleCommand extends CommandBase {
  private final DriveBase driveBase;
  private final Rotation2d targetAngle;
  private double turnOutput;
  private final PIDController pidController =
      new PIDController(PathConstants.TURN_P, PathConstants.TURN_I, PathConstants.TURN_D);


  public DriveBaseTurnToAngleCommand(DriveBase driveBase, Rotation2d targetAngle, double angleTolerance) {
    this.driveBase = driveBase;
    this.targetAngle = targetAngle;
    pidController.enableContinuousInput(0, 360);
    pidController.setTolerance(angleTolerance);
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    pidController.setSetpoint(targetAngle.getDegrees());
  }

  @Override
  public void execute() {
    turnOutput = pidController.calculate(driveBase.getGyroAngle().getDegrees());
    driveBase.tankDrive(turnOutput, -turnOutput, false);
  }

  @Override
  public boolean isFinished() {
    return pidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }
}
