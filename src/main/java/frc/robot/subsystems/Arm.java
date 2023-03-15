
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmPose;
import frc.robot.subsystems.arm.Elevator;
import frc.robot.subsystems.arm.Pivot;

public class Arm extends SubsystemBase {
  private final Pivot pivot;
  private final Elevator elevator;

  public Arm(Pivot pivot, Elevator elevator) {
    this.pivot = pivot;
    this.elevator = elevator;
  }

  public Pivot getPivot() {
    return this.pivot;
  }

  public Elevator getElevator() {
    return this.elevator;
  }

  public ArmPose getPose() {
    return ArmPose.fromAngleExtension(this.pivot.getRotation(), this.elevator.getExtension());
  }

  public ArmPose getSetpointPose() {
    return ArmPose.fromAngleExtension(Rotation2d.fromDegrees(this.pivot.getSetpoint().position),
        this.elevator.getSetpointExtension());
  }
}
