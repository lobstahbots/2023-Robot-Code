
package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.PathConstants;
import frc.robot.subsystems.DriveBase;

public class TurnToAngleCommand extends CommandBase {
  private final DriveBase driveBase;
  private final Rotation2d targetAngle;
  private double turnOutput;
  private final PIDController pidController = new PIDController(PathConstants.kP, PathConstants.kI, PathConstants.KD);


  public TurnToAngleCommand(DriveBase driveBase, Rotation2d targetAngle, double angleTolerance) {
    this.driveBase = driveBase;
    this.targetAngle = targetAngle;
    pidController.enableContinuousInput(-180, 180);
    pidController.setTolerance(angleTolerance);
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    pidController.setSetpoint(targetAngle.getDegrees());
  }

  @Override
  public void execute() {
    turnOutput = pidController.calculate(driveBase.getHeading().getDegrees());
    driveBase.tankDrive(-turnOutput, turnOutput, false);
    SmartDashboard.putNumber("Turn Output", turnOutput);
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
