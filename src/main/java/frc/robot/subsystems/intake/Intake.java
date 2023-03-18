// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/**
 * A subsystem that controls the intaking/outtaking mechanism on the robot.
 */
public class Intake extends SubsystemBase {
  private final MotorControllerGroup intakeMotors;
  private final PowerDistribution powerDistribution;
  private final int leftIntakePdID;
  private final int rightIntakePdID;

  /**
   * Creates an Intake with a {@link PWMSparkMax} at the given motor IDs.
   * 
   * @param leftIntakeMotorID The ID of the left intake motor
   * @param rightIntakeMotorID The ID of the right intake motor
   * @param powerDistribution The {@link PowerDistribution} to use to monitor the motors
   * @param leftIntakePdID The ID of the {@link PowerDistribution} channel for the left intake motor
   * @param rightIntakePdID The ID of the {@link PowerDistribution} channel for the right intake motor
   */
  public Intake(int leftIntakeMotorID, int rightIntakeMotorID,
      PowerDistribution powerDistribution, int leftIntakePdID, int rightIntakePdID) {
    intakeMotors = new MotorControllerGroup(
        new PWMSparkMax(leftIntakeMotorID),
        new PWMSparkMax(rightIntakeMotorID));

    this.powerDistribution = powerDistribution;
    this.leftIntakePdID = leftIntakePdID;
    this.rightIntakePdID = rightIntakePdID;
  }

  /**
   * Sets the spin speed of the intake motors.
   * 
   * @param speed The speed to set the motors to.
   */
  public void setSpinSpeed(double speed) {
    intakeMotors.set(speed);
  }

  /**
   * Sets the spin voltage of the intake motors.
   * 
   * @param voltage The voltage to set the motors to.
   */
  public void setSpinVoltage(double voltage) {
    intakeMotors.setVoltage(voltage);
  }

  /**
   * Returns whether the left intake motor is stopped/stalled.
   */
  public boolean getLeftStopped() {
    return powerDistribution.getCurrent(leftIntakePdID) > IntakeConstants.STOPPED_CURRENT_THRESHOLD;
  }

  /**
   * Returns whether the right intake motor is stopped/stalled.
   */
  public boolean getRightStopped() {
    return powerDistribution.getCurrent(rightIntakePdID) > IntakeConstants.STOPPED_CURRENT_THRESHOLD;
  }

  /**
   * Returns whether both sides of the intake are stopped/stalled.
   */
  public boolean getBothStopped() {
    return getLeftStopped() && getRightStopped();
  }

  /**
   * Returns whether only one side of the intake is stopped/stalled.
   */
  public boolean getOnlyOneStopped() {
    return getLeftStopped() ^ getRightStopped();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Has Game Piece", getBothStopped());
    SmartDashboard.putBoolean("One Side Stopped", getOnlyOneStopped());
  }

  public void stopMotors() {
    intakeMotors.stopMotor();
  }

}
