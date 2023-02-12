// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax elevatorMotor;
  private final DigitalInput limitSwitch;
  private final TrapezoidProfile.Constraints constraints =
      new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocity, ElevatorConstants.kMaxAcceleration);
  private final ProfiledPIDController pidController =
      new ProfiledPIDController(ElevatorConstants.kP, 0, 0, constraints);
  private final Encoder encoder;

  public Elevator(int elevatorMotorID, int encoderChannelA, int encoderChannelB, int limitSwitchChannel) {
    this.elevatorMotor = new CANSparkMax(elevatorMotorID, MotorType.kBrushless);
    elevatorMotor.setInverted(true);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    this.encoder = new Encoder(encoderChannelA, encoderChannelB, true, Encoder.EncodingType.k1X);
    encoder.setDistancePerPulse(ElevatorConstants.kDistancePerPulse);
    this.limitSwitch = new DigitalInput(limitSwitchChannel);
  }

  public void setTargetExtension(double position) {
    pidController.setGoal(position);
  }

  public boolean isRetracted() {
    return !this.limitSwitch.get();
  }

  public void move(double speed) {
    elevatorMotor.set(speed);
  }

  public double getExtension() {
    return encoder.getDistance();
  }

  public double getLength() {
    return ElevatorConstants.LENGTH_FULLY_RETRACTED + encoder.getDistance();
  }

  @Override
  public void periodic() {
    System.out.println(this.getExtension());
    SmartDashboard.putNumber("Elevator Extension", this.getExtension());
  }


}