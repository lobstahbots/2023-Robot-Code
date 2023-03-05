
package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.PivotConstants;
import frc.robot.ArmPose;
import frc.robot.Constants.ArmConstants.ElevatorConstants;

public class Arm extends SubsystemBase {
  private final Pivot pivot;
  private final Elevator elevator;

  public Arm(Pivot pivot, Elevator elevator) {
    this.pivot = pivot;
    this.elevator = elevator;
  }

  public Pivot getPivot() {
    return this.pivot;
  }

  public Elevator getElevator() {
    return this.elevator;
  }

  public ArmPose getPose() {
    return ArmPose.fromAngleExtension(this.pivot.getRotation(), this.elevator.getExtension());
  }

  public ArmPose getSetpointPose() {
    return ArmPose.fromAngleExtension(Rotation2d.fromDegrees(this.pivot.getSetpoint()),
        this.elevator.getSetpointExtension());
  }

  /**
   * A sub-subsystem that controls the rotating pivot on the robot.
   */
  public static class Pivot {
    private final DutyCycleEncoder pivotEncoder;
    private final CANSparkMax leftPivotMotor;
    private final CANSparkMax rightPivotMotor;
    private final MotorControllerGroup motors;
    private final ArmFeedforward feedforward =
        new ArmFeedforward(
            PivotConstants.S_VOLTS, PivotConstants.G_VOLTS,
            PivotConstants.V_VOLT_SECOND_PER_RAD, PivotConstants.A_VOLT_SECOND_SQUARED_PER_RAD);
    private final Constraints constraints = new TrapezoidProfile.Constraints(
        PivotConstants.MAX_VELOCITY_DEG_PER_SEC,
        PivotConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED);
    private final ProfiledPIDController pidController = new ProfiledPIDController(PivotConstants.P, 0, 0, constraints);

    /**
     * Constructs an Pivot with an {@link CANSparkMax} at the motor IDs and {@link DutyCycleEncoder} at the encoder
     * channel.
     * 
     * @param leftMotorID The ID of the left pivot motor
     * @param rightMotorID The ID of the right pivot motor
     * @param encoderChannel The encoder channel on the RIO
     */
    public Pivot(int leftMotorID, int rightMotorID, int encoderChannel) {
      this.leftPivotMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
      this.rightPivotMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
      leftPivotMotor.setIdleMode(IdleMode.kBrake);
      rightPivotMotor.setIdleMode(IdleMode.kBrake);
      leftPivotMotor.setInverted(false);
      rightPivotMotor.setInverted(true);
      leftPivotMotor.setSmartCurrentLimit(PivotConstants.CURRENT_LIMIT);
      rightPivotMotor.setSmartCurrentLimit(PivotConstants.CURRENT_LIMIT);
      motors = new MotorControllerGroup(leftPivotMotor, rightPivotMotor);

      this.pivotEncoder = new DutyCycleEncoder(new DigitalInput(encoderChannel));
      pivotEncoder.setDistancePerRotation(PivotConstants.PIVOT_DEGREES_PER_ROTATION);

      pidController.setTolerance(PivotConstants.ROTATION_PID_TOLERANCE);
    }

    /**
     * Sets the braking mode to the given {@link IdleMode}.
     *
     * @param mode The {@link IdleMode} to set the motors to
     */
    public void setIdleMode(IdleMode mode) {
      leftPivotMotor.setIdleMode(mode);
      rightPivotMotor.setIdleMode(mode);
    }

    /**
     * Gets angle of the pivot.
     * 
     * @return The rotation of the pivot in degrees. 0 = Vertical and pointing down. Positive -> towards front of robot.
     */
    public double getAngle() {
      return PivotConstants.PIVOT_OFFSET_DEG - pivotEncoder.getAbsolutePosition() * 360;
    }

    /**
     * Gets a {@link Rotation2d} representing the current angle of the pivot.
     * 
     * @return An {@link Rotation2d} of the rotation of the pivot. 0 = Vertical and pointing down. Positive -> towards
     *         front of robot.
     */
    public Rotation2d getRotation() {
      return Rotation2d.fromDegrees(getAngle());
    }

    /**
     * Gets PID setpoint of the pivot.
     * 
     * @return The setpoint angle of the pivot in degrees. 0 = Vertical and pointing down. Positive -> towards front of
     *         robot.
     */
    public double getSetpoint() {
      return pidController.getSetpoint().position;
    }

    /**
     * Sets rotation speed of the pivot motors. Includes stop to keep pivot from rotating beyond limits.
     * 
     * @param speed The desired rotation speed
     */
    public void setRotationSpeed(double speed) {
      if (getAngle() < PivotConstants.MIN_ROTATION_DEG && motors.get() < 0
          || getAngle() > PivotConstants.MAX_ROTATION_DEG && motors.get() > 0) {
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
     * Returns whether the pivot is at the setpoint / within the deadband.
     * 
     * @return Whether the pivot is at the PID goal angle.
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
  }

  /**
   * A sub-subsystem that controls the extending elevator on the robot.
   */
  public static class Elevator {
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
    }

    /**
     * Sets the braking mode to the given {@link IdleMode}.
     *
     * @param mode The {@link IdleMode} to set the motors to
     */
    public void setIdleMode(IdleMode mode) {
      elevatorMotor.setIdleMode(mode);
    }

    /**
     * Sets the speed of the elevator motor. Includes a stop to keep elevator from extending or retracting beyond
     * limits.
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
     * Moves elevator until it triggers the limit switch, ignoring retraction limits. Only used to reset elevator
     * encoder.
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
  }

}
