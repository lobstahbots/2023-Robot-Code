
package frc.robot.subsystems.arm;

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
import frc.robot.Constants.ArmConstants.PivotConstants;

/**
 * A sub-subsystem that controls the rotating pivot on the robot.
 */
public class Pivot {
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
