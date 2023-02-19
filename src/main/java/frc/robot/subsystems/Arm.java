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
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

/**
 * A subsystem that controls the rotating arm on the robot.
 */
public class Arm extends SubsystemBase {

  private final DutyCycleEncoder armEncoder;
  private final CANSparkMax leftArmMotor;
  private final CANSparkMax rightArmMotor;
  private final MotorControllerGroup motors;
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.kSVolts, ArmConstants.kGVolts,
          ArmConstants.kVVoltSecondPerRad, ArmConstants.kAVoltSecondSquaredPerRad);
  private final Constraints constraints = new TrapezoidProfile.Constraints(
      ArmConstants.MAX_VELOCITY_DEG_PER_SEC,
      ArmConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED);
  private final ProfiledPIDController pidController = new ProfiledPIDController(ArmConstants.kP, 0, 0, constraints);

  /**
   * Constructs an Arm with an {@link CANSparkMax} at the motor IDs and {@link DutyCycleEncoder} at the encoder channel.
   * 
   * @param leftMotorID The ID of the left arm motor
   * @param rightMotorID The ID of the right arm motor
   * @param encoderChannel The encoder channel on the RIO
   */
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
    motors = new MotorControllerGroup(leftArmMotor, rightArmMotor);
    armEncoder.setDistancePerRotation(ArmConstants.ARM_DEGREES_PER_ROTATION);
    pidController.setTolerance(ArmConstants.ROTATION_ERROR_DEADBAND);
  }

  /**
   * Gets angle of the arm. The arm's fully retracted position is 0 degrees. Note: This is not vertical.
   * 
   * @return The rotation of the arm in degrees.
   */
  public double getRotation() {
    return ArmConstants.ARM_OFFSET_DEG - armEncoder.getAbsolutePosition() * 360;
  }

  /**
   * Sets rotation speed of the arm motors. Includes stop to keep arm from rotating beyond limits.
   * 
   * @param speed The desired rotation speed
   */
  public void setRotationSpeed(double speed) {
    if (getRotation() < ArmConstants.MIN_ROTATION_DEG && motors.get() < 0
        || getRotation() > ArmConstants.MAX_ROTATION_DEG && motors.get() > 0) {
      motors.set(0);
      return;
    }
    motors.set(speed);
  }

  /**
   * Sets setpoint angle of the PID controller.
   * 
   * @param goalAngle The desired setpoint in degrees.
   */
  public void setPIDGoal(double goalAngle) {
    pidController.setGoal(goalAngle);
    pidController.reset(getRotation());

  }

  /**
   * Returns whether the arm is at the setpoint / within the deadband.
   * 
   * @return Whether the arm is at the PID goal angle.
   */
  public boolean atSetpoint() {
    return pidController.atGoal();
  }

  /**
   * Feeds the PID input to the motors.
   */
  public void feedPID() {
    setRotationSpeed(-pidController.calculate(getRotation()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Rotation", getRotation());
  }
}
