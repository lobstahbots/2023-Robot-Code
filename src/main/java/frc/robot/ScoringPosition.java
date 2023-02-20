
package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.ScoringSystemConstants.*;

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

  public static ScoringPosition fromArmElevator(Rotation2d armAngle, double elevatorExtension) {
    return new ScoringPosition(armAngle, elevatorExtension);
  }

  /**
   * 
   * Constructs a ScoringPosition with a {@link Translation2d} relative to the scoring coordinate origin.
   * 
   * @param position The position of the scoring position relative to the scoring origin.
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
   * 
   * Constructs a ScoringPosition with x and y coordinates relative to the scoring coordinate origin.
   * 
   * @param x The x coordinate of the scoring position relative to the scoring origin.
   * @param y The y coordinate of the scoring position relative to the scoring origin.
   */
  public static ScoringPosition fromXY(double x, double y) {
    return fromXY(new Translation2d(x, y));
  }

  public Rotation2d getArmAngle() {
    return armAngle;
  }

  public double getElevatorExtension() {
    return elevatorExtension;
  }

  public Translation2d getXY() {
    return position;
  }

  public double getX() {
    return position.getX();
  }

  public double getY() {
    return position.getY();
  }

  public ScoringPosition rotateArmBy(Rotation2d rotation) {
    return fromArmElevator(armAngle.rotateBy(rotation), elevatorExtension);
  }

  public ScoringPosition extendElevatorBy(double extension) {
    return fromArmElevator(armAngle, elevatorExtension + extension);
  }

  public ScoringPosition translateBy(Translation2d translation) {
    return fromXY(position.plus(translation));
  }
}
