
package frc.robot.subsystems.arm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ArmPose;
import frc.robot.Constants.ArmConstants.ElevatorConstants;
import frc.robot.Constants.ArmConstants.PivotConstants;

public class Arm extends SubsystemBase {
  private final DutyCycleEncoder pivotEncoder;
  private final CANSparkMax pivotMotor;
  private final CANSparkMax followerPivotMotor;

  private final ArmFeedforward armFeedforward =
      new ArmFeedforward(
          PivotConstants.S_VOLTS, PivotConstants.G_VOLTS,
          PivotConstants.V_VOLT_SECOND_PER_RAD, PivotConstants.A_VOLT_SECOND_SQUARED_PER_RAD);
  private final ProfiledPIDController pivotPIDController =
      new ProfiledPIDController(PivotConstants.P, 0, PivotConstants.D, new TrapezoidProfile.Constraints(
          PivotConstants.MAX_VELOCITY_DEG_PER_SEC,
          PivotConstants.MAX_ACCELERATION_DEG_PER_SEC_SQUARED));

  private final CANSparkMax elevatorMotor;
  private final DigitalInput limitSwitch;
  private final PIDController elevatorPIDController = new PIDController(ElevatorConstants.P, 0, 0);
  private final Encoder encoder;

  public Arm(int leftPivotMotorID, int rightPivotMotorID, int pivotEncoderChannel, int elevatorMotorID,
      int elevatorEncoderChannelA, int elevatorEncoderChannelB, int limitSwitchChannel) {
    this.pivotMotor = new CANSparkMax(leftPivotMotorID, MotorType.kBrushless);
    pivotMotor.setSmartCurrentLimit(PivotConstants.CURRENT_LIMIT);

    this.followerPivotMotor = new CANSparkMax(rightPivotMotorID, MotorType.kBrushless);
    followerPivotMotor.setSmartCurrentLimit(PivotConstants.CURRENT_LIMIT);
    followerPivotMotor.follow(pivotMotor, true);

    this.pivotEncoder = new DutyCycleEncoder(new DigitalInput(pivotEncoderChannel));
    pivotEncoder.setDistancePerRotation(PivotConstants.PIVOT_DEGREES_PER_ROTATION);

    pivotPIDController.setTolerance(PivotConstants.ROTATION_PID_TOLERANCE);

    this.elevatorMotor = new CANSparkMax(elevatorMotorID, MotorType.kBrushless);
    elevatorMotor.setIdleMode(IdleMode.kBrake);
    elevatorMotor.setInverted(false);
    elevatorMotor.setSmartCurrentLimit(ElevatorConstants.CURRENT_LIMIT);
    this.encoder = new Encoder(elevatorEncoderChannelA, elevatorEncoderChannelB, true, Encoder.EncodingType.k1X);
    encoder.setDistancePerPulse(ElevatorConstants.DISTANCE_PER_PULSE);
    this.limitSwitch = new DigitalInput(limitSwitchChannel);

    setIdleMode(IdleMode.kBrake);
  }

  public ArmPose getPose() {
    return ArmPose.fromAngleExtension(getRotation(), getExtension());
  }

  public ArmPose getSetpointPose() {
    return ArmPose.fromAngleExtension(Rotation2d.fromDegrees(getPivotSetpoint()),
        getElevatorSetpoint());
  }

  /**
   * Sets the braking mode to the given {@link IdleMode}.
   *
   * @param mode The {@link IdleMode} to set the motors to
   */
  public void setIdleMode(IdleMode mode) {
    pivotMotor.setIdleMode(mode);
    followerPivotMotor.setIdleMode(mode);
    elevatorMotor.setIdleMode(mode);
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
   * @return The setpoint state of the pivot with position in degrees and velocity in degrees per second. 0 = Vertical
   *         and pointing down. Positive -> towards front of robot.
   */
  public State getPivotSetpointState() {
    return pivotPIDController.getGoal();
  }

  /**
   * Gets PID setpoint of the pivot.
   * 
   * @return The setpoint angle of the pivot in degrees. 0 = Vertical and pointing down. Positive -> towards front of
   *         robot.
   */
  public double getPivotSetpoint() {
    return getPivotSetpointState().position;
  }

  /**
   * Sets rotation speed of the pivot motors. Includes stop to keep pivot from rotating beyond limits.
   * 
   * @param speed The desired rotation speed
   */
  public void setPivotSpeed(double speed) {
    if (getAngle() < PivotConstants.MIN_ROTATION_DEG && pivotMotor.get() < 0
        || getAngle() > PivotConstants.MAX_ROTATION_DEG && pivotMotor.get() > 0) {
      pivotMotor.set(0);
      return;
    }
    pivotMotor.set(speed);
  }


  /**
   * Sets spin voltage of the pivot motors. Includes stop to keep pivot from rotating beyond limits.
   * 
   * @param voltage The desired voltage
   */
  public void setPivotVoltage(double voltage) {
    if (getAngle() < PivotConstants.MIN_ROTATION_DEG && pivotMotor.get() < 0
        || getAngle() > PivotConstants.MAX_ROTATION_DEG && pivotMotor.get() > 0) {
      pivotMotor.set(0);
      return;
    }
    pivotMotor.set(voltage);
  }

  /**
   * Resets PID controller error.
   */
  public void resetPivotPID() {
    pivotPIDController.reset(getAngle());
  }

  /**
   * Sets setpoint angle of the PID controller.
   * 
   * @param goalAngle The desired setpoint in degrees.
   */
  public void setPivotPIDGoal(double goalAngle) {
    pivotPIDController.setGoal(goalAngle);
  }

  /**
   * Returns whether the pivot is at the setpoint / within the deadband.
   * 
   * @return Whether the pivot is at the PID goal angle.
   */
  public boolean atPivotSetpoint() {
    return pivotPIDController.atGoal();
  }

  /**
   * Feeds the PID input to the motors.
   */
  public void feedPivotPID() {
    // setPivotSpeed(pivotPIDController.calculate(getAngle()));
    useOutput(pivotPIDController.calculate(getAngle()), getPivotSetpointState());
  }

  /**
   * Sets the voltage of the motors based on calculated output from PID controller and feedforward.
   */
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    double feedforward = armFeedforward.calculate(Math.toRadians(setpoint.position), setpoint.velocity);
    setPivotVoltage(output + feedforward);
  }

  /**
   * Sets the speed of the elevator motor. Includes a stop to keep elevator from extending or retracting beyond limits.
   * 
   * @param speed The desired extension/retraction speed.
   */
  public void setElevatorSpeed(double speed) {
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
  public void resetElevatorPID() {
    elevatorPIDController.reset();
  }

  /**
   * Sets setpoint extension of the PID controller.
   * 
   * @param goalExtension The desired setpoint extension in inches.
   */
  public void setElevatorPIDGoal(double goalExtension) {
    elevatorPIDController.setSetpoint(goalExtension);
  }

  /**
   * Feeds the PID input to the motor.
   */
  public void feedElevatorPID() {
    this.setElevatorSpeed(elevatorPIDController.calculate(this.getExtension()));
  }

  /**
   * Determines whether the elevator is retracted based on the limit switch value.
   * 
   * @return Whether the elevator is retracted enough to trigger the limit switch
   */
  public boolean isElevatorRetracted() {
    return !this.limitSwitch.get();
  }

  /**
   * Moves elevator until it triggers the limit switch, ignoring retraction limits. Only used to reset elevator encoder.
   */
  public void moveElevatorToLimitSwitch() {
    if (!isElevatorRetracted()) {
      elevatorMotor.set(-ElevatorConstants.HOME_SPEED);
    }
  }

  /**
   * Resets the elevator encoder
   */
  public void resetElevatorEncoder() {
    this.encoder.reset();
  }

  /**
   * Gets the setpoint extension of the elevator.
   * 
   * @return The PID controller goal extension of the elevator in inches
   */
  public double getElevatorSetpoint() {
    return elevatorPIDController.getSetpoint();
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
