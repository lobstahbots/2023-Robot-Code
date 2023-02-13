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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {

  private final DutyCycleEncoder armEncoder;
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private final Constraints constraints = new TrapezoidProfile.Constraints(
      ArmConstants.kMaxVelocityRadPerSecond,
      ArmConstants.kMaxAccelerationRadPerSecSquared);
  private final ProfiledPIDController pidController = new ProfiledPIDController(ArmConstants.kP, 0, 0, constraints);

  public Arm(int leftMotorID, int rightMotorID, int encoderChannel) {
    this.leftArmMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    this.rightArmMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    this.armEncoder = new DutyCycleEncoder(new DigitalInput(encoderChannel));
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setInverted(true);
    rightArmMotor.setInverted(false);
    leftArmMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
    rightArmMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
    armEncoder.setDistancePerRotation(ArmConstants.ARM_DEGREES_PER_ROTATION);
    SmartDashboard.putData("Arm PID", this.pidController);
  }

  public double getAngle() {
    return ArmConstants.kArmOffsetDeg - armEncoder.getAbsolutePosition() * 360;
  }

  public void setRotationSpeed(double speed) {
    leftArmMotor.set(speed);
    rightArmMotor.set(speed);
  }

  public void setPIDGoal(double goalAngle) {
    pidController.setGoal(goalAngle);
    this.setRotationSpeed(-pidController.calculate(this.getAngle()));
  }

  public void feedPID() {
    this.setRotationSpeed(-pidController.calculate(this.getAngle()));
  }

  public void feedForwardPID(TrapezoidProfile.State setpoint) {
    this.leftArmMotor.setVoltage(-pidController.calculate(this.getAngle()) + feedforward.calculate(Units.degreesToRadians(setpoint.position - ArmConstants.STRAIGHT_ARM_OFFSET), setpoint.velocity));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Rotation", this.getAngle());
  }

}
