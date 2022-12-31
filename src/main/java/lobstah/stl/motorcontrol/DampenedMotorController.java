
package lobstah.stl.motorcontrol;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

/**
 * A wrapper that adds a slew rate limit to a motor controller.
 */
public class DampenedMotorController implements MotorController {
  private final MotorController controller;
  private SlewRateLimiter dampener;

  /*
   * This stores the last calculated speed from the dampener because the SlewRateLimiter doesn't have a way of
   * retrieving this value.
   * 
   * TODO: https://github.com/wpilibsuite/allwpilib/pull/4883
   */
  private double lastSpeed;

  /**
   * Creates a new DampenedMotorController with the given rate limits.
   *
   * @param controller The motor controller to wrap.
   * @param positiveRateLimit The rate-of-change limit in the positive direction, in decimaL percent per second. This is
   *          expected to be positive.
   * @param negativeRateLimit The rate-of-change limit in the negative direction, in decimal percent per second. This is
   *          expected to be negative.
   */
  public DampenedMotorController(MotorController controller, double positiveRateLimit, double negativeRateLimit) {
    this.controller = controller;
    this.lastSpeed = controller.get();
    this.dampener = new SlewRateLimiter(positiveRateLimit, negativeRateLimit, lastSpeed);
  }

  /**
   * Creates a new DampenedMotorController with the given rate limit.
   *
   * @param controller The motor controller to wrap.
   * @param rateLimit The rate-of-change limit in decimal percent per second. This is expected to be positive.
   */
  public DampenedMotorController(MotorController controller, double rateLimit) {
    this(controller, rateLimit, -rateLimit);
  }

  /**
   * Creates a new DampenedMotorController with an (effectively) infinite rate limit.
   * 
   * @param controller The motor controller to wrap.
   */
  public DampenedMotorController(MotorController controller) {
    this(controller, 999999999);
  }

  /**
   * Sets a new rate limit for the motor controller.
   * 
   * TODO: Add support for different positive and negative rate limits.
   * 
   * @param rateLimit The maximum rate of change in output percentage per second.
   */
  public void setRateLimit(double rateLimit) {
    dampener = new SlewRateLimiter(rateLimit, -rateLimit, lastSpeed);
  }

  /**
   * Sets the output of the motor controller, clamped to [-1, 1], and limited by the saved rate limit.
   * 
   * Must be called regularly if the input speed is far away from the current speed or the rate limit is low.
   * 
   * @param speed The output percentage.
   */
  @Override
  public void set(double speed) {
    lastSpeed = dampener.calculate(MathUtil.clamp(speed, -1, 1));
    controller.set(lastSpeed);
  }

  @Override
  public double get() {
    return controller.get();
  }

  /**
   * Inverts the movement direction of the motor controller. Inversion will not take effect until the next time set() is
   * called, and it will respect the rate limit.
   */
  @Override
  public void setInverted(boolean isInverted) {
    /*
     * Inverts the motor controller, and then sets the speed of the motor controller to the last speed, but inverted.
     * This allows the motor controller to respect the rate limit, and not perform a full speed invert immediately.
     */

    if (isInverted != controller.getInverted()) {
      lastSpeed = -lastSpeed;
      dampener.reset(lastSpeed);
      controller.stopMotor();
      controller.setInverted(isInverted);
      controller.set(lastSpeed);
    }


  }

  @Override
  public boolean getInverted() {
    return controller.getInverted();
  }

  /**
   * Disable the motor controller.
   * 
   * Also stops the motor at full speed. This is a safety feature, as disable() is more explicit than set(0).
   */
  @Override
  public void disable() {
    dampener.reset(0);
    lastSpeed = 0;
    controller.disable();
  }

  /**
   * Stops the motor at full speed. This is a safety feature, as stopMotor() is more explicit than set(0).
   */
  @Override
  public void stopMotor() {
    dampener.reset(0);
    lastSpeed = 0;
    controller.stopMotor();
  }
}
