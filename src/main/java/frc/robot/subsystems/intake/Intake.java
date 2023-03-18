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
   * Gets the maximum current in amperes of the two intake motors.
   */
  public double getMaxCurrent() {
    return Math.max(powerDistribution.getCurrent(leftIntakePdID), powerDistribution.getCurrent(rightIntakePdID));
  }

  /**
   * Returns whether the intake is stopped/stalled.
   */
  public boolean getStopped() {
    return getMaxCurrent() < IntakeConstants.STOPPED_CURRENT_THRESHOLD;
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Intake stopped", getStopped());
  }
  public void stopMotors() {
    intakeMotors.stopMotor();
  }

}
