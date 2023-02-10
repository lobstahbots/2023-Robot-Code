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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ArmConstants;

public class Arm extends ProfiledPIDSubsystem {

  private final DutyCycleEncoder armEncoder;
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);

  public Arm(int leftMotorID, int rightMotorID, int encoderChannel) {
    super(new ProfiledPIDController(
        ArmConstants.kP,
        0,
        0,
        new TrapezoidProfile.Constraints(
            ArmConstants.kMaxVelocityRadPerSecond,
            ArmConstants.kMaxAccelerationRadPerSecSquared)));
    this.leftArmMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    this.rightArmMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    this.armEncoder = new DutyCycleEncoder(new DigitalInput(encoderChannel));
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setInverted(true);
    rightArmMotor.setInverted(false);
    armEncoder.setDistancePerRotation(ArmConstants.ARM_DEGREES_PER_ROTATION);
    setGoal(ArmConstants.kArmOffsetDeg);

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    leftArmMotor.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
    rightArmMotor.setVoltage(output + feedforward.calculate(setpoint.position, setpoint.velocity));
  }

  @Override
  public double getMeasurement() {
    return armEncoder.getDistance() + ArmConstants.kArmOffsetDeg;
  }

  public void setRotationSpeed(double speed) {
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Rotation", this.getMeasurement());
  }

}
