// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final CANSparkMax leftIntakeMotor;
  private final CANSparkMax rightIntakeMotor;

  public Intake(int leftIntakeMotorID, int rightIntakeMotorID) {
    this.leftIntakeMotor = new CANSparkMax(leftIntakeMotorID, MotorType.kBrushless);
    this.rightIntakeMotor = new CANSparkMax(rightIntakeMotorID, MotorType.kBrushless);
    leftIntakeMotor.setIdleMode(IdleMode.kBrake);
    rightIntakeMotor.setIdleMode(IdleMode.kBrake);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public void setSpinSpeed(double speed) {
    leftIntakeMotor.set(speed);
    rightIntakeMotor.set(speed);
  }

}
