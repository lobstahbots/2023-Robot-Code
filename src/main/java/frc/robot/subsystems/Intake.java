// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final PWMSparkMax leftIntakeMotor;
  private final PWMSparkMax rightIntakeMotor;

  public Intake(int leftIntakeMotorID, int rightIntakeMotorID) {
    this.leftIntakeMotor = new PWMSparkMax(leftIntakeMotorID);
    this.rightIntakeMotor = new PWMSparkMax(rightIntakeMotorID);
  }

  public void setSpinSpeed(double speed) {
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }

}
