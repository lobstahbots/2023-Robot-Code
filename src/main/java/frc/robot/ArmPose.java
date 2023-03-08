
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants.ArmConstants;

import static frc.robot.Constants.ArmConstants.*;

/** Represents a possible pose of the arm (pivot and elevator). */
public class ArmPose {
  private final Rotation2d angle;
  private final double extension;
  private final Translation2d position;

  private ArmPose(Rotation2d angle, double extension) {
    this.angle = Rotation2d.fromDegrees(
        MathUtil.clamp(angle.getDegrees(), PivotConstants.MIN_ROTATION_DEG, PivotConstants.MAX_ROTATION_DEG));
    this.extension = MathUtil.clamp(extension, ElevatorConstants.MIN_EXTENSION_INCHES,
        ElevatorConstants.MAX_EXTENSION_INCHES);

    this.position = new Translation2d(
        -Constants.IntakeConstants.INTAKE_OFFSET.getY(),
        -ElevatorConstants.LENGTH_FULLY_RETRACTED - extension - Constants.IntakeConstants.INTAKE_OFFSET.getX())
            .rotateBy(angle).plus(PivotConstants.ORIGIN_TO_PIVOT);
  }

  /**
   * Constructs an ArmPose with the provided pivot angle and elevator extension.
   * 
   * @param angle The angle of the pivot. 0 = Vertical and pointing down. Positive -> towards front of robot. This will
   *          be clamped to the range of the pivot.
   * @param extension The extension of the elevator in inches. This will be clamped to the range of the elevator.
   */
  public static ArmPose fromAngleExtension(Rotation2d angle, double extension) {
    return new ArmPose(angle, extension);
  }

  /**
   * Constructs an ArmPose with an intake position relative to the scoring coordinate origin. Will be clamped to the
   * range of the pivot and elevator.
   * 
   * @param position The position of the intake relative to the scoring origin, in inches.
   */
  public static ArmPose fromXY(Translation2d position) {
    Translation2d relativeToPivot = position.minus(PivotConstants.ORIGIN_TO_PIVOT);

    // Angle between the arm and the line intersecting the pivot and intake position
    Rotation2d angleFromHypotenuse =
        new Rotation2d(Math.asin(Constants.IntakeConstants.INTAKE_OFFSET.getY() / relativeToPivot.getNorm()));
    System.out.println(angleFromHypotenuse.getDegrees());
    Rotation2d pivotAngle = relativeToPivot.getAngle().plus(Rotation2d.fromDegrees(90)).plus(angleFromHypotenuse);
    // .plus(Rotation2d.fromDegrees(10));

    // Intake position relative to the pivot, with X axis aligned to the arm
    Translation2d alignedToArm = new Translation2d(relativeToPivot.getNorm(), angleFromHypotenuse);
    double length = alignedToArm.getX();
    double extension =
        length - ElevatorConstants.LENGTH_FULLY_RETRACTED - Constants.IntakeConstants.INTAKE_OFFSET.getX();

    return new ArmPose(pivotAngle, extension);
  }

  /**
   * Constructs an ArmPose with x and y intake coordinates relative to the scoring coordinate origin. Will be clamped to
   * the range of the pivot and elevator.
   * 
   * @param x The x coordinate of the intake relative to the scoring origin, in inches.
   * @param y The y coordinate of the intake relative to the scoring origin, in inches.
   */
  public static ArmPose fromXY(double x, double y) {
    return fromXY(new Translation2d(x, y));
  }

  /**
   * Gets the angle of the pivot at the ArmPose.
   * 
   * @return The angle of the pivot. 0 = Vertical and pointing down. Positive -> towards front of robot.
   */
  public Rotation2d getAngle() {
    return angle;
  }

  /**
   * Gets the extension of the elevator at the ArmPose.
   * 
   * @return The extension of the elevator in inches.
   */
  public double getExtension() {
    return extension;
  }

  /**
   * Gets the position of the intake at the ArmPose.
   * 
   * @return The position of the intake relative to the scoring coordinate origin, in inches.
   */
  public Translation2d getXY() {
    return position;
  }

  /**
   * Gets the x coordinate of the intake at the ArmPose.
   * 
   * @return The x coordinate of the intake relative to the scoring coordinate origin, in inches.
   */
  public double getX() {
    return position.getX();
  }

  /**
   * Gets the y coordinate of the intake at the ArmPose.
   * 
   * @return The y coordinate of the intake relative to the scoring coordinate origin, in inches.
   */
  public double getY() {
    return position.getY();
  }

  /**
   * Gets the distance between the intake positions of two ArmPoses.
   * 
   * @param other The ArmPose to compute the distance to
   * @return The distance between the intake positions of the two ArmPose, in inches.
   */
  public double getDistance(ArmPose other) {
    return position.getDistance(other.position);
  }

  /**
   * @param rotation The amount to rotate the pivot by. Positive -> towards front of robot when pointing down.
   * 
   * @return A new ArmPose with the pivot rotated by the given rotation.
   */
  public ArmPose rotateBy(Rotation2d rotation) {
    return fromAngleExtension(angle.rotateBy(rotation), extension);
  }

  /**
   * @param extension The amount to extend the elevator by.
   * @return A new ArmPose with the elevator extended by the given amount.
   */
  public ArmPose extendBy(double extension) {
    return fromAngleExtension(angle, extension + extension);
  }

  /**
   * @param translation The amount to translate the intake by, in inches.
   * @return A new ArmPose with the intake position translated by the given amount.
   */
  public ArmPose translateBy(Translation2d translation) {
    return fromXY(position.plus(translation));
  }

  /**
   * @return Whether the ArmPose is within the bumper collision zone.
   */
  public boolean isInsideBumperZone() {
    return getAngle().getDegrees() < ArmConstants.BUMPER_AVOIDANCE_ANGLE.getDegrees()
        && getX() > ArmConstants.BUMPER_AVOIDANCE_X;
  }
}
