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
import frc.robot.Constants.ScoringSystemConstants.ElevatorConstants;

/**
 * A subsystem that controls the extending elevator on the robot.
 */
public class Elevator extends SubsystemBase {
  private final CANSparkMax elevatorMotor;
  private final DigitalInput limitSwitch;
  private final PIDController pidController = new PIDController(ElevatorConstants.P, 0, 0);
  private final Encoder encoder;

  /**
   * Constructs an Elevator with a {@link CANSparkMax} at the motor ID, an {@link Encoder} with the provided encoder
   * channels, and {@link DigitalInput} limit switch at the given channel.
   * 
   * @param elevatorMotorID The ID of the elevator motor
   * @param encoderChannelA The encoder channel A
   * @param encoderChannelB The encoder channel B
   * @param limitSwitchChannel The channel of the digital input to the limit switch
   */
  public Elevator(int elevatorMotorID, int encoderChannelA, int encoderChannelB, int limitSwitchChannel) {
    this.elevatorMotor = new CANSparkMax(elevatorMotorID, MotorType.kBrushless);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setInverted(false);
    elevatorMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
    this.encoder = new Encoder(encoderChannelA, encoderChannelB, true, Encoder.EncodingType.k1X);
    encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);
    this.limitSwitch = new DigitalInput(limitSwitchChannel);
    pidController.setTolerance(0.3);
  }

  /**
   * Sets the speed of the elevator motor. Includes a stop to keep elevator from extending or retracting beyond limits.
   * 
   * @param speed The desired extension/retraction speed.
   */
  public void move(double speed) {
    if (getExtension() > ElevatorConstants.MAX_EXTENSION_INCHES && speed > 0
        || getExtension() < ElevatorConstants.MIN_EXTENSION_INCHES && speed < 0) {
      elevatorMotor.set(0.0);
      return;
    }
    elevatorMotor.set(speed);
  }

  /**
   * Resets PID controller error.
   * 
   */
  public void resetPID() {
    pidController.reset();
  }

  /**
   * Sets setpoint extension of the PID controller.
   * 
   * @param goalExtension The desired setpoint extension in inches.
   */
  public void setPIDGoal(double goalExtension) {
    pidController.setSetpoint(goalExtension);
  }

  /**
   * Feeds the PID input to the motor.
   */
  public void feedPID() {
    this.move(pidController.calculate(this.getExtension()));
  }

  /**
   * Determines whether the elevator is retracted based on the limit switch value.
   * 
   * @return Whether the elevator is retracted enough to trigger the limit switch
   */
  public boolean isRetracted() {
    return !this.limitSwitch.get();
  }

  /**
   * Moves elevator until it triggers the limit switch, ignoring retraction limits. Only used to reset elevator encoder.
   */
  public void moveToLimitSwitch() {
    if (!isRetracted()) {
      elevatorMotor.set(-ElevatorConstants.HOME_SPEED);
    }
  }

  /**
   * Resets the elevator encoder
   */
  public void resetEncoder() {
    this.encoder.reset();
  }

  /**
   * Gets the setpoint extension of the elevator.
   * 
   * @return The PID controller goal extension of the elevator in inches
   */
  public double getSetpointExtension() {
    return pidController.getSetpoint();
  }

  /**
   * Gets the current extension of the elevator
   * 
   * @return The extension of the elevator in inches
   */
  public double getExtension() {
    return encoder.getDistance();
  }

  /**
   * Gets the actual current length of the elevator
   * 
   * @return The full length of the elevator in inches.
   */
  public double getLength() {
    return ElevatorConstants.LENGTH_FULLY_RETRACTED + getExtension();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Elevator Extension", getExtension());
  }
}
