// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ScoringSystemConstants.ArmConstants;

/**
 * A subsystem that controls the rotating arm on the robot.
 */
public class Arm extends SubsystemBase {

  private final DutyCycleEncoder armEncoder;
  private final MotorControllerGroup motors;
  private final ArmFeedforward feedforward =
      new ArmFeedforward(
          ArmConstants.S_VOLTS, ArmConstants.G_VOLTS,
          ArmConstants.V_VOLT_SECOND_PER_RAD, ArmConstants.A_VOLT_SECOND_SQUARED_PER_RAD);
  private final Constraints constraints = new TrapezoidProfile.Constraints(
      ArmConstants.MAX_VELOCITY_DEG_PER_SEC,
      ArmConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED);
  private final ProfiledPIDController pidController = new ProfiledPIDController(ArmConstants.P, 0, 0, constraints);

  /**
   * Constructs an Arm with an {@link CANSparkMax} at the motor IDs and {@link DutyCycleEncoder} at the encoder channel.
   * 
   * @param leftMotorID The ID of the left arm motor
   * @param rightMotorID The ID of the right arm motor
   * @param encoderChannel The encoder channel on the RIO
   */
  public Arm(int leftMotorID, int rightMotorID, int encoderChannel) {
    CANSparkMax leftArmMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
    CANSparkMax rightArmMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setInverted(false);
    rightArmMotor.setInverted(true);
    leftArmMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
    rightArmMotor.setSmartCurrentLimit(ArmConstants.CURRENT_LIMIT);
    motors = new MotorControllerGroup(leftArmMotor, rightArmMotor);

    this.armEncoder = new DutyCycleEncoder(new DigitalInput(encoderChannel));
    armEncoder.setDistancePerRotation(ArmConstants.ARM_DEGREES_PER_ROTATION);

    pidController.setTolerance(ArmConstants.ROTATION_PID_TOLERANCE);
  }

  /**
   * Gets angle of the arm.
   * 
   * @return The rotation of the arm in degrees. 0 = Vertical and pointing down. Positive -> towards front of robot.
   */
  public double getAngle() {
    return ArmConstants.ARM_OFFSET_DEG - armEncoder.getAbsolutePosition() * 360;
  }

  /**
   * Gets a {@link Rotation2d} representing the current angle of the arm.
   * 
   * @return An {@link Rotation2d} of the rotation of the arm. 0 = Vertical and pointing down. Positive -> towards front
   *         of robot.
   */
  public Rotation2d getRotation() {
    return Rotation2d.fromDegrees(getAngle());
  }

  /**
   * Gets PID setpoint of the arm.
   * 
   * @return The setpoint angle of the arm in degrees. 0 = Vertical and pointing down. Positive -> towards front of
   *         robot.
   */
  public double getSetpoint() {
    return pidController.getSetpoint().position;
  }

  /**
   * Sets rotation speed of the arm motors. Includes stop to keep arm from rotating beyond limits.
   * 
   * @param speed The desired rotation speed
   */
  public void setRotationSpeed(double speed) {
    if (getAngle() < ArmConstants.MIN_ROTATION_DEG && motors.get() < 0
        || getAngle() > ArmConstants.MAX_ROTATION_DEG && motors.get() > 0) {
      motors.set(0);
      return;
    }
    motors.set(speed);
  }


  /**
   * Resets PID controller error.
   */
  public void resetPID() {
    pidController.reset(getAngle());
  }

  /**
   * Sets setpoint angle of the PID controller.
   * 
   * @param goalAngle The desired setpoint in degrees.
   */
  public void setPIDGoal(double goalAngle) {
    pidController.setGoal(goalAngle);
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
    setRotationSpeed(pidController.calculate(getAngle()));
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Rotation", getAngle());
  }
}
