// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends ProfiledPIDSubsystem {

  private final Encoder armEncoder;
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  public Arm(int leftMotorID, int rightMotorID, int encoderChannelA, int encoderChannelB) {
    super(new ProfiledPIDController(
        ArmConstants.kP,
        0,
        0,
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond,
            ArmConstants.kMaxAccelerationRadPerSecSquared)),
        0);
    this.leftArmMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    this.rightArmMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    this.armEncoder = new Encoder(encoderChannelA, encoderChannelB);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setInverted(false);
    rightArmMotor.setInverted(true);
    armEncoder.setDistancePerPulse(ArmConstants.kEncoderDistancePerPulse);
    setGoal(ArmConstants.kArmOffsetRads);


    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    leftArmMotor.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
    rightArmMotor.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
  }

  public double getMeasurement() {
    return armEncoder.getDistance() + ArmConstants.kArmOffsetRads;
  }

  public void setRotationSpeed(double speed) {
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

}
