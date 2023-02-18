// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final CANSparkMax elevatorMotor;
  private final DigitalInput limitSwitch;
  private final PIDController pidController = new PIDController(ElevatorConstants.kP, 0, 0);
  private final Encoder encoder;

  public Elevator(int elevatorMotorID, int encoderChannelA, int encoderChannelB, int limitSwitchChannel) {
    this.elevatorMotor = new CANSparkMax(elevatorMotorID, MotorType.kBrushless);
    elevatorMotor.setInverted(true);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
    this.encoder = new Encoder(encoderChannelA, encoderChannelB, true, Encoder.EncodingType.k1X);
    encoder.setDistancePerPulse(ElevatorConstants.kDistancePerPulse);
    this.limitSwitch = new DigitalInput(limitSwitchChannel);
    // SmartDashboard.putData("Elevator PID", this.pidController);
  }

  public void setPIDGoal(double goalExtension) {
    pidController.setSetpoint(goalExtension);
  }

  public void feedPID() {
    this.move(-pidController.calculate(this.getExtension()));
  }

  public boolean isRetracted() {
    return !this.limitSwitch.get();
  }

  public void move(double speed) {
    if (getExtension() > ElevatorConstants.kMaxExtension && speed < 0) {
      elevatorMotor.set(0.0);
      return;
    }
    if (getExtension() < ElevatorConstants.kMinExtension && speed > 0) {
      elevatorMotor.set(0.0);
      return;
    }
    elevatorMotor.set(speed);
  }

  public void moveToSwitch() {
    if (!isRetracted()) {
      elevatorMotor.set(ElevatorConstants.RETRACT_SPEED);
    }
  }

  public double getExtension() {
    return encoder.getDistance();
  }

  public double getLength() {
    return ElevatorConstants.LENGTH_FULLY_RETRACTED + encoder.getDistance();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Elevator Extension", this.getExtension());
  }

  public void resetEncoder() {
    this.encoder.reset();
  }


}
