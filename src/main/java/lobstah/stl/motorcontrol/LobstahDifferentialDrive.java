
package lobstah.stl.motorcontrol;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * An extension of the {@link DifferentialDrive} class that adds slew rate limiting.
 */
public class LobstahDifferentialDrive extends DifferentialDrive {
  private final DampenedMotorController left;
  private final DampenedMotorController right;


  private LobstahDifferentialDrive(DampenedMotorController left, DampenedMotorController right) {
    super(left, right);
    this.left = left;
    this.right = right;
  }

  /**
   * Creates a new LobstahDifferentialDrive with an (effectively) infinite rate limit.
   *
   * @param left The left motor controller.
   * @param right The right motor controller.
   */
  public LobstahDifferentialDrive(MotorController left, MotorController right) {
    this(new DampenedMotorController(left), new DampenedMotorController(right));
  }

  /**
   * Creates a new LobstahDifferentialDrive with the given rate limit.
   *
   * @param left The left motor controller.
   * @param right The right motor controller.
   * @param rateLimit The maximum rate of change in output percentage per second.
   */
  public LobstahDifferentialDrive(MotorController left, MotorController right, double rateLimit) {
    this(new DampenedMotorController(left, rateLimit), new DampenedMotorController(right, rateLimit));
  }

  /**
   * Tank drive method for differential drive platform using voltage control.
   * 
   * @param leftVolts The left motor output voltage.
   * @param rightVolts The right motor output voltage.
   */
  public void tankDriveVoltage(double leftVolts, double rightVolts) {
    left.setVoltage(leftVolts);
    right.setVoltage(rightVolts);
    feed();
  }

  /**
   * Sets a new rate limit for the motor controllers.
   * 
   * @param rateLimit The maximum rate of change in output percentage per second.
   */
  public void setRateLimit(double rateLimit) {
    left.setRateLimit(rateLimit);
    right.setRateLimit(rateLimit);
  }

}

