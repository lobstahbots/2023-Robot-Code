
package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import static frc.robot.Constants.ScoringSystemConstants.*;

public class ScoringPosition extends Translation2d {
  /**
   * 
   * Constructs a ScoringPosition with x and y coordinates relative to the scoring coordinate origin.
   * 
   * @param x The x coordinate of the scoring position relative to the scoring origin.
   * @param y The y coordinate of the scoring position relative to the scoring origin.
   */
  public ScoringPosition(double x, double y) {
    super(x, y);
  }

  /**
   * 
   * Constructs a ScoringPosition with a {@link Translation2d} relative to the scoring coordinate origin.
   * 
   * @param position The position of the scoring position relative to the scoring origin.
   */
  public ScoringPosition(Translation2d position) {
    this(position.getX(), position.getY());
  }

  /**
   * Constructs a ScoringPosition with a {@link Translation2d} relative to the arm pivot.
   *
   * @param positionFromPivot The position of the scoring position relative to the arm pivot.
   */
  public static ScoringPosition fromPivot(Translation2d positionFromPivot) {
    return new ScoringPosition(positionFromPivot.plus(ArmConstants.ORIGIN_TO_PIVOT));
  }

  public Translation2d toPivot() {
    return this.minus(ArmConstants.ORIGIN_TO_PIVOT);
  }

  public static ScoringPosition fromArmElevator(Rotation2d armAngle, double elevatorExtension) {
    return fromPivot(new Translation2d(
        IntakeConstants.INTAKE_OFFSET.getY(),
        ElevatorConstants.LENGTH_FULLY_RETRACTED + elevatorExtension + IntakeConstants.INTAKE_OFFSET.getX())
            .rotateBy(armAngle));
  }

  public Rotation2d getArmAngle() {
    return this.toPivot().getAngle()
        .plus(new Rotation2d(Math.asin(IntakeConstants.INTAKE_OFFSET.getY() / this.toPivot().getNorm())));
  }

  public double getElevatorExtension() {
    double distance = this.toPivot().getNorm();
    double length = Math.sqrt(Math.pow(distance, 2) - Math.pow(IntakeConstants.INTAKE_OFFSET.getY(), 2));
    return length - ElevatorConstants.LENGTH_FULLY_RETRACTED - IntakeConstants.INTAKE_OFFSET.getX();
  }
}
