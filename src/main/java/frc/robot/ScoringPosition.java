
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.ScoringSystemConstants.*;

/** Represents a possible position of the scoring system (arm, elevator, and intake). */
public class ScoringPosition {
  private final Rotation2d armAngle;
  private final double elevatorExtension;
  private final Translation2d position;

  private ScoringPosition(Rotation2d armAngle, double elevatorExtension) {
    this.armAngle = new Rotation2d(
        MathUtil.clamp(armAngle.getDegrees(), ArmConstants.MIN_ROTATION_DEG, ArmConstants.MAX_ROTATION_DEG));
    this.elevatorExtension = MathUtil.clamp(elevatorExtension, ElevatorConstants.MIN_EXTENSION_INCHES,
        ElevatorConstants.MAX_EXTENSION_INCHES);

    this.position = new Translation2d(
        IntakeConstants.INTAKE_OFFSET.getY(),
        ElevatorConstants.LENGTH_FULLY_RETRACTED + elevatorExtension + IntakeConstants.INTAKE_OFFSET.getX())
            .rotateBy(armAngle).plus(ArmConstants.ORIGIN_TO_PIVOT);
  }

  /**
   * Constructs a ScoringPosition with the provided arm angle and elevator extension.
   * 
   * @param armAngle The angle of the arm. 0 = Vertical and pointing down. Positive -> towards front of robot. This will
   *          be clamped to the range of the arm.
   * @param elevatorExtension The extension of the elevator in inches. This will be clamped to the range of the
   *          elevator.
   */
  public static ScoringPosition fromArmElevator(Rotation2d armAngle, double elevatorExtension) {
    return new ScoringPosition(armAngle, elevatorExtension);
  }

  /**
   * Constructs a ScoringPosition with an intake position relative to the scoring coordinate origin. Will be clamped to
   * the range of the arm and elevator.
   * 
   * @param position The position of the intake relative to the scoring origin, in inches.
   */
  public static ScoringPosition fromXY(Translation2d position) {
    Translation2d relativeToPivot = position.minus(ArmConstants.ORIGIN_TO_PIVOT);
    Rotation2d armAngle = relativeToPivot.getAngle()
        .plus(new Rotation2d(Math.asin(IntakeConstants.INTAKE_OFFSET.getY() / relativeToPivot.getNorm())));

    double length =
        Math.sqrt(Math.pow(relativeToPivot.getNorm(), 2) - Math.pow(IntakeConstants.INTAKE_OFFSET.getY(), 2));
    double elevatorExtension = length - ElevatorConstants.LENGTH_FULLY_RETRACTED - IntakeConstants.INTAKE_OFFSET.getX();

    return new ScoringPosition(armAngle, elevatorExtension);
  }

  /**
   * Constructs a ScoringPosition with x and y intake coordinates relative to the scoring coordinate origin. Will be
   * clamped to the range of the arm and elevator.
   * 
   * @param x The x coordinate of the intake relative to the scoring origin, in inches.
   * @param y The y coordinate of the intake relative to the scoring origin, in inches.
   */
  public static ScoringPosition fromXY(double x, double y) {
    return fromXY(new Translation2d(x, y));
  }

  /**
   * Gets the angle of the arm at the ScoringPosition.
   * 
   * @return The angle of the arm. 0 = Vertical and pointing down. Positive -> towards front of robot.
   */
  public Rotation2d getArmAngle() {
    return armAngle;
  }

  /**
   * Gets the extension of the elevator at the ScoringPosition.
   * 
   * @return The extension of the elevator in inches.
   */
  public double getElevatorExtension() {
    return elevatorExtension;
  }

  /**
   * Gets the position of the intake at the ScoringPosition.
   * 
   * @return The position of the intake relative to the scoring coordinate origin, in inches.
   */
  public Translation2d getXY() {
    return position;
  }

  /**
   * Gets the x coordinate of the intake at the ScoringPosition.
   * 
   * @return The x coordinate of the intake relative to the scoring coordinate origin, in inches.
   */
  public double getX() {
    return position.getX();
  }

  /**
   * Gets the y coordinate of the intake at the ScoringPosition.
   * 
   * @return The y coordinate of the intake relative to the scoring coordinate origin, in inches.
   */
  public double getY() {
    return position.getY();
  }

  /**
   * @param rotation The amount to rotate the arm by. Positive -> towards front of robot when pointing down.
   * 
   * @return A new ScoringPosition with the arm rotated by the given rotation.
   */
  public ScoringPosition rotateArmBy(Rotation2d rotation) {
    return fromArmElevator(armAngle.rotateBy(rotation), elevatorExtension);
  }

  /**
   * @param extension The amount to extend the elevator by.
   * @return A new ScoringPosition with the elevator extended by the given amount.
   */
  public ScoringPosition extendElevatorBy(double extension) {
    return fromArmElevator(armAngle, elevatorExtension + extension);
  }

  /**
   * @param translation The amount to translate the intake by, in inches.
   * @return A new ScoringPosition with the intake position translated by the given amount.
   */
  public ScoringPosition translateBy(Translation2d translation) {
    return fromXY(position.plus(translation));
  }
}
