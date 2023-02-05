// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax elevatorMotor;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(1.75, 0.75);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(ElevatorConstants.kP, 0, 0, constraints, 0);
  private final Encoder encoder;

  public Elevator(int elevatorMotorID, int encoderChannelA, int encoderChannelB) {
    this.elevatorMotor = new CANSparkMax(elevatorMotorID, MotorType.kBrushless);
    elevatorMotor.setInverted(true);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    this.encoder = new Encoder(encoderChannelA, encoderChannelB);
    encoder.setDistancePerPulse(1.0 / 360.0 * 2.0 * Math.PI * 1.5);
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public void setGoal(int position) {
    pidController.setGoal(position);
  }

  @Override
  public void periodic() {
    elevatorMotor.set(pidController.calculate(encoder.getDistance()));
  }

  public double getExtension() {
    return 0;
  }
}
