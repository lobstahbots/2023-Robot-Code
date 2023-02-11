
package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.List;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import lobstah.stl.math.LobstahMath;
import lobstah.stl.motorcontrol.LobstahDifferentialDrive;
import org.photonvision.EstimatedRobotPose;

/**
 * A subsystem that controls the drive train (aka chassis) on a robot.
 */
public class DriveBase extends SubsystemBase {

  private final WPI_TalonFX leftFrontMotor;
  private final WPI_TalonFX leftBackMotor;
  private final WPI_TalonFX rightFrontMotor;
  private final WPI_TalonFX rightBackMotor;

  private NeutralMode motorNeutralMode;

  private final LobstahDifferentialDrive differentialDrive;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final PhotonVision photonVision;
  private final AHRS gyro = new AHRS();

  /**
   * Constructs a DriveBase with a {@link TalonFX} at each of the given CAN IDs.
   *
   * @param leftFrontId The CAN ID of the Left Front motor
   * @param leftBackId The CAN ID of the Left Back motor
   * @param rightFrontId The CAN ID of the Right Front motor
   * @param rightBackId The CAN ID of the Right Back motor
   */
  public DriveBase(int leftFrontId, int leftBackId, int rightFrontId, int rightBackId) {
    leftFrontMotor = new WPI_TalonFX(leftFrontId);
    leftFrontMotor.setInverted(TalonFXInvertType.CounterClockwise);
    leftBackMotor = new WPI_TalonFX(leftBackId);
    leftBackMotor.setInverted(TalonFXInvertType.CounterClockwise);

    rightFrontMotor = new WPI_TalonFX(rightFrontId);
    rightFrontMotor.setInverted(TalonFXInvertType.Clockwise);
    rightBackMotor = new WPI_TalonFX(rightBackId);
    rightBackMotor.setInverted(TalonFXInvertType.Clockwise);

    leftFrontMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.SUPPLY_CURRENT_LIMIT,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD_TIME));
    leftBackMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.SUPPLY_CURRENT_LIMIT,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD_TIME));
    rightFrontMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.SUPPLY_CURRENT_LIMIT,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD_TIME));
    rightBackMotor.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(true, DriveConstants.SUPPLY_CURRENT_LIMIT,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD,
            DriveConstants.SUPPLY_TRIGGER_THRESHOLD_TIME));

    leftFrontMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, DriveConstants.STATOR_CURRENT_LIMIT,
            DriveConstants.STATOR_TRIGGER_THRESHOLD,
            DriveConstants.STATOR_TRIGGER_THRESHOLD_TIME));
    leftBackMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, DriveConstants.STATOR_CURRENT_LIMIT,
            DriveConstants.STATOR_TRIGGER_THRESHOLD,
            DriveConstants.STATOR_TRIGGER_THRESHOLD_TIME));
    rightFrontMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, DriveConstants.STATOR_CURRENT_LIMIT,
            DriveConstants.STATOR_TRIGGER_THRESHOLD,
            DriveConstants.STATOR_TRIGGER_THRESHOLD_TIME));
    rightBackMotor.configStatorCurrentLimit(
        new StatorCurrentLimitConfiguration(true, DriveConstants.STATOR_CURRENT_LIMIT,
            DriveConstants.STATOR_TRIGGER_THRESHOLD,
            DriveConstants.STATOR_TRIGGER_THRESHOLD_TIME));


    leftFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    leftBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightFrontMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
    rightBackMotor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);


    setNeutralMode(NeutralMode.Brake);

    differentialDrive =
        new LobstahDifferentialDrive(
            new MotorControllerGroup(leftFrontMotor, leftBackMotor),
            new MotorControllerGroup(rightFrontMotor, rightBackMotor),
            DriveConstants.ACCELERATION_RATE_LIMIT);

    resetEncoders();
    poseEstimator =
        new DifferentialDrivePoseEstimator(DriveConstants.KINEMATICS, gyro.getRotation2d(), 0, 0, new Pose2d());

    this.photonVision = new PhotonVision();

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Toggles the {@link NeutralMode} between Coast and Brake.
   */
  public void toggleNeutralMode() {
    switch (motorNeutralMode) {
      case Brake:
        setNeutralMode(NeutralMode.Coast);
        return;
      case Coast:
        setNeutralMode(NeutralMode.Brake);
        return;
      default:
        setNeutralMode(NeutralMode.Brake);
        return;
    }
  }

  /**
   * Sets the neutral mode to the given {@link NeutralMode}.
   *
   * @param mode The {@link NeutralMode} to set the motors to
   */
  public void setNeutralMode(NeutralMode mode) {
    leftFrontMotor.setNeutralMode(mode);
    leftBackMotor.setNeutralMode(mode);
    rightFrontMotor.setNeutralMode(mode);
    rightBackMotor.setNeutralMode(mode);
    motorNeutralMode = mode;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    poseEstimator.update(getHeading(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
    // Pair<Pose3d, Double> visionEstimatedPose = photonVision.getEstimatedGlobalPose();
    // if (visionEstimatedPose.getFirst() != null) {
    // poseEstimator.addVisionMeasurement(visionEstimatedPose.getFirst().toPose2d(), visionEstimatedPose.getSecond());
    // }
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        LobstahMath.nativeUnitsToVelocityMetersPerSecond(leftFrontMotor.getSelectedSensorVelocity()),
        LobstahMath.nativeUnitsToVelocityMetersPerSecond(rightFrontMotor.getSelectedSensorVelocity()));
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param translation2d The translation from the origin to use when creating a {@link Pose2d} to reset the odometry.
   * @param rotation The gyro angle to use when creating a {@link Pose2d} to reset the odometry.
   */
  public void resetOdometry(Translation2d translation2d, Rotation2d rotation) {
    gyro.reset();
    poseEstimator.resetPosition(new Rotation2d(), 0, 0, new Pose2d(translation2d, rotation));
    resetEncoders();
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    leftFrontMotor.setSelectedSensorPosition(0);
    leftBackMotor.setSelectedSensorPosition(0);
    rightFrontMotor.setSelectedSensorPosition(0);
    rightBackMotor.setSelectedSensorPosition(0);
  }

  /**
   * Gets the distance of the left encoder in meters.
   */
  public double getLeftEncoderDistanceMeters() {
    return LobstahMath.nativeUnitsToDistanceMeters(leftFrontMotor.getSelectedSensorPosition()) / 2;
  }

  /**
   * Gets the distance of the right encoder in meters.
   */
  public double getRightEncoderDistanceMeters() {
    return LobstahMath.nativeUnitsToDistanceMeters(rightBackMotor.getSelectedSensorPosition()) / 2;
  }

  /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
  public double getAverageEncoderDistanceMeters() {
    return (getLeftEncoderDistanceMeters() + getRightEncoderDistanceMeters()) / 2.0;
  }

  /** Zeroes the gyro value. */
  public void zeroGyro() {
    gyro.reset();
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVoltage(double leftVolts, double rightVolts) {
    differentialDrive.tankDriveVoltage(leftVolts, rightVolts);
  }

  /**
   * Returns the heading of the robot from 180 to -180 degrees in radians.
   *
   * @return the robot's heading in radians as a Rotation2d.
   */
  public Rotation2d getHeading() {
    return new Rotation2d(Math.toRadians(gyro.getYaw()));
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return gyro.getRate();
  }

  /**
   * Returns the Transform2d from the pose of the robot to the target Pose.
   */
  public Transform2d getDistanceToPose(Pose2d targetPose) {
    return this.getPose().minus(targetPose);
  }

  /**
   * Generates a trajectory through a list of provided waypoints from the robot's position to the given target Pose.
   * 
   * @return A PathPlannerTrajectory to follow to the target position.
   */
  public PathPlannerTrajectory generatePath(List<Pose2d> waypoints) {
    ArrayList<PathPoint> pathPoints = new ArrayList<>();
    pathPoints.add(new PathPoint(this.getPose().getTranslation(), this.getPose().getRotation()));
    for (Pose2d waypoint : waypoints) {
      pathPoints.add(new PathPoint(waypoint.getTranslation(), waypoint.getRotation()));
    }
    return PathPlanner
        .generatePath(new PathConstraints(PathConstants.MAX_DRIVE_SPEED, PathConstants.MAX_ACCELERATION), pathPoints);
  }

  /**
   * Sets the motor speeds to 0.
   */
  public void stopDrive() {
    differentialDrive.stopMotor();
  }

  /**
   * Drives the motors using arcade drive controls.
   *
   * @param linearSpeed The linear speed
   * @param angularSpeed The angular speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public void arcadeDrive(double linearSpeed, double angularSpeed, boolean squaredInputs) {
    differentialDrive.arcadeDrive(linearSpeed, angularSpeed, squaredInputs);
  }

  /**
   * Drives the motors using tank drive controls.
   *
   * @param leftSpeed The left speed
   * @param rightSpeed The right speed
   * @param squaredInputs Whether to drive with squared inputs
   */
  public void tankDrive(double leftSpeed, double rightSpeed, boolean squaredInputs) {
    differentialDrive.tankDrive(leftSpeed, rightSpeed, squaredInputs);
  }

  @Override
  /**
   * Updates the Pose Estimator with measurements from Photonvision and odometry and writes relevant values to the
   * Shuffleboard.
   */
  public void periodic() {
    poseEstimator.update(getHeading(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
    try {
      List<EstimatedRobotPose> visionEstimatedPoses = new ArrayList<>();
      Pose2d estimatedVisionPose = new Pose2d();
      Rotation2d averageEstimatedRotation = new Rotation2d();
      double averageEstimatedX = 0;
      double averageEstimatedY = 0;
      double averageTimestamp = 0;
      for (EstimatedRobotPose pose : photonVision.getEstimatedGlobalPoses()) {
        if (pose.estimatedPose.toPose2d().minus(poseEstimator.getEstimatedPosition()).getTranslation()
            .getNorm() < PathConstants.MAX_VISION_TO_ODOMETRY_ERROR) {
          visionEstimatedPoses.add(pose);
        }
      }

      for (EstimatedRobotPose pose : visionEstimatedPoses) {
        averageEstimatedRotation = averageEstimatedRotation.plus(pose.estimatedPose.toPose2d().getRotation());
        averageEstimatedX += pose.estimatedPose.toPose2d().getX();
        averageEstimatedY += pose.estimatedPose.toPose2d().getY();
        averageTimestamp += pose.timestampSeconds;
      }

      averageEstimatedRotation = averageEstimatedRotation.div(visionEstimatedPoses.size());
      averageEstimatedX /= visionEstimatedPoses.size();
      averageEstimatedY /= visionEstimatedPoses.size();
      averageTimestamp /= visionEstimatedPoses.size();
      estimatedVisionPose = new Pose2d(averageEstimatedX, averageEstimatedY, averageEstimatedRotation);

      SmartDashboard.putString("PhotonVision Pose", estimatedVisionPose.toString());
      poseEstimator.addVisionMeasurement(estimatedVisionPose, averageTimestamp);

    } catch (NullPointerException npe) {

    }

    SmartDashboard.putNumber("Gyro", this.getHeading().getDegrees());
    SmartDashboard.putString("Pose", this.getPose().toString());
    SmartDashboard.putNumber("Number of Tags Visible In Front", this.photonVision.getFrontTargets().size());
    SmartDashboard.putNumber("Number of Tags Visible In Rear", this.photonVision.getRearTargets().size());
  }
}
