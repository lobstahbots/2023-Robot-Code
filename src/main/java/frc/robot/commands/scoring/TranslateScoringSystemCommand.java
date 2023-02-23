// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.scoring;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ScoringPosition;
import frc.robot.Constants.ScoringSystemConstants.ArmConstants;
import frc.robot.Constants.ScoringSystemConstants.ElevatorConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class TranslateScoringSystemCommand extends CommandBase {
  private final Arm arm;
  private final Elevator elevator;
  private final DoubleSupplier xTranslation;
  private final DoubleSupplier yTranslation;

  /**
   * Creates a command that moves the {@link Arm} and {@link Elevator} towards a given position.
   *
   * @param arm The {@link Arm} to control
   * @param elevator The {@link Elevator} to control
   * @param position The position to move towards
   */
  public TranslateScoringSystemCommand(Arm arm, Elevator elevator,
      double xTranslation, double yTranslation) {
    this(arm, elevator, () -> xTranslation, () -> yTranslation);
  }

  public TranslateScoringSystemCommand(Arm arm, Elevator elevator, DoubleSupplier xTranslation,
      DoubleSupplier yTranslation) {
    this.arm = arm;
    this.elevator = elevator;
    this.xTranslation = xTranslation;
    this.yTranslation = yTranslation;
    addRequirements(arm, elevator);
  }

  @Override
  public void initialize() {
    arm.resetPID();
    elevator.resetPID();
  }

  @Override
  public void execute() {
    ScoringPosition newPosition =
        ScoringPosition.fromArmElevator(Rotation2d.fromDegrees(arm.getSetpoint()), elevator.getSetpointExtension())
            .translateBy(new Translation2d(xTranslation.getAsDouble(), yTranslation.getAsDouble()));

    double clampedAngle =
        MathUtil.clamp(newPosition.getArmAngle().getDegrees(), ArmConstants.MIN_ROTATION_DEG,
            ArmConstants.MAX_ROTATION_DEG);

    double clampedExtension = MathUtil.clamp(newPosition.getElevatorExtension(), ElevatorConstants.MIN_EXTENSION_INCHES,
        ElevatorConstants.MAX_EXTENSION_INCHES);

    SmartDashboard.putNumber("Translating Extension", clampedExtension);
    SmartDashboard.putNumber("Translating Rotation", clampedAngle);
    SmartDashboard.putString("Translating Position", newPosition.getXY().toString());

    this.elevator.setPIDGoal(clampedExtension);
    this.elevator.feedPID();
    arm.setPIDGoal(clampedAngle);
    arm.feedPID();
  }

  @Override
  public void end(boolean interrupted) {
    arm.setRotationSpeed(0);
    elevator.move(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
