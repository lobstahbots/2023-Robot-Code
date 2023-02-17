// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final MotorControllerGroup intakeMotors;

  public Intake(int leftIntakeMotorID, int rightIntakeMotorID) {
    intakeMotors = new MotorControllerGroup(
        new PWMSparkMax(leftIntakeMotorID),
        new PWMSparkMax(rightIntakeMotorID));
  }

  public void setSpinSpeed(double speed) {
    intakeMotors.set(speed);
  }

  public void setSpinVoltage(double voltage) {
    intakeMotors.setVoltage(voltage);
  }

}
