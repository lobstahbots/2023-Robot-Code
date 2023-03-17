// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A subsystem that controls the intaking/outtaking mechanism on the robot.
 */
public class Intake extends SubsystemBase {
  private final MotorControllerGroup intakeMotors;

  /**
   * Creates an Intake with a {@link PWMSparkMax} at the given motor IDs.
   * 
   * @param leftIntakeMotorID The ID of the left intake motor
   * @param rightIntakeMotorID The ID of the right intake motor
   */
  public Intake(int leftIntakeMotorID, int rightIntakeMotorID) {
    intakeMotors = new MotorControllerGroup(
        new PWMSparkMax(leftIntakeMotorID),
        new PWMSparkMax(rightIntakeMotorID));
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

  public void stopMotors() {
    intakeMotors.stopMotor();
  }

}
