
package frc.robot.commands.drive;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import lobstah.stl.math.LobstahMath;

public class TurnToAngleCommand extends CommandBase {
  DriveBase driveBase;
  Rotation2d targetAngle;
  double turnSpeed;
  double turnOutput;

  public TurnToAngleCommand(DriveBase driveBase, Rotation2d targetAngle, double turnSpeed) {
    this.driveBase = driveBase;
    this.targetAngle = targetAngle;
    this.turnSpeed = turnSpeed;
    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    this.turnOutput = Math.copySign(
        this.turnSpeed,
        LobstahMath.calculateTurningOutput(
            driveBase.getHeading().getDegrees(),
            this.targetAngle.getDegrees()));
  }

  @Override
  public void execute() {
    driveBase.tankDrive(-turnOutput, turnOutput, false);
    SmartDashboard.putNumber("Turn Output", turnOutput);
  }

  @Override
  public boolean isFinished() {
    return Math.signum(LobstahMath.calculateTurningOutput(
        driveBase.getHeading().getDegrees(),
        this.targetAngle.getDegrees())) * Math.signum(turnOutput) < 0;
  }

  @Override
  public void end(boolean interrupted) {
    driveBase.stopDrive();
  }
}
