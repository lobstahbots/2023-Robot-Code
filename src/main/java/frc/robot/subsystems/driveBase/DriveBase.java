
package frc.robot.subsystems.driveBase;

import java.lang.reflect.Field;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
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
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PathConstants;
import frc.robot.Constants.VisionConstants;
import lobstah.stl.math.LobstahMath;
import lobstah.stl.motorcontrol.LobstahDifferentialDrive;
import frc.robot.subsystems.photonvision.EstimatedRobotPose;
import frc.robot.subsystems.photonvision.PhotonVision;

/**
 * A subsystem that controls the drive train (aka chassis) on a robot.
 */
public class DriveBase extends SubsystemBase {

  private final WPI_TalonFX leftFrontMotor;
  private final WPI_TalonFX leftBackMotor;
  private final WPI_TalonFX rightFrontMotor;
  private final WPI_TalonFX rightBackMotor;

  private final MotorControllerGroup leftMotors;
  private final MotorControllerGroup rightMotors;

  private final TalonFXSimCollection simLeftFrontMotor;
  private final TalonFXSimCollection simRightFrontMotor;
  private final TalonFXSimCollection simLeftBackMotor;
  private final TalonFXSimCollection simRightBackMotor;

  private NeutralMode motorNeutralMode;

  private final LobstahDifferentialDrive differentialDrive;
  private final DifferentialDrivePoseEstimator poseEstimator;
  private final PhotonVision photonVision;
  private final AHRS gyro = new AHRS();
  private boolean hasSeenTag = false;
  private boolean gyroInitialized = false;

  public DifferentialDrivetrainSim drivetrainSimulator;
  private final Field2d fieldSim;

  private final SimDouble gyroAngle;

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

    this.leftMotors = new MotorControllerGroup(leftFrontMotor, leftBackMotor);
    this.rightMotors = new MotorControllerGroup(rightFrontMotor, rightBackMotor);


    setNeutralMode(NeutralMode.Brake);

    differentialDrive =
        new LobstahDifferentialDrive(
            leftMotors,
            rightMotors,
            DriveConstants.ACCELERATION_RATE_LIMIT);

    resetEncoders();
    poseEstimator =
        new DifferentialDrivePoseEstimator(DriveConstants.KINEMATICS, getGyroAngle180(), 0, 0, new Pose2d());
    this.photonVision = new PhotonVision();

    if (RobotBase.isSimulation()) { // If our robot is simulated
      // This class simulates our drivetrain's motion around the field.
      // drivetrainSimulator =
      // new DifferentialDrivetrainSim(
      // DriveConstants.kDrivetrainPlant,
      // DriveConstants.kDriveGearbox,
      // DriveConstants.kDriveGearing,
      // Units.inchesToMeters(DriveConstants.TRACK_WIDTH),
      // Units.inchesToMeters(DriveConstants.WHEEL_RADIUS_INCHES),
      // VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
      drivetrainSimulator = DifferentialDrivetrainSim.createKitbotSim(
          KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
          KitbotGearing.k10p71, // 10.71:1
          KitbotWheelSize.kSixInch, // 6" diameter wheels.
          null // No measurement noise.
      );

      simLeftBackMotor = new TalonFXSimCollection(leftBackMotor);
      simRightBackMotor = new TalonFXSimCollection(rightBackMotor);
      simLeftFrontMotor = new TalonFXSimCollection(leftFrontMotor);
      simRightFrontMotor = new TalonFXSimCollection(rightFrontMotor);

      simLeftFrontMotor.setSupplyCurrent(DriveConstants.SUPPLY_CURRENT_LIMIT);
      simRightFrontMotor.setSupplyCurrent(DriveConstants.SUPPLY_CURRENT_LIMIT);
      simLeftBackMotor.setSupplyCurrent(DriveConstants.SUPPLY_CURRENT_LIMIT);
      simRightBackMotor.setSupplyCurrent(DriveConstants.SUPPLY_CURRENT_LIMIT);
      simLeftFrontMotor.setStatorCurrent(DriveConstants.STATOR_CURRENT_LIMIT);
      simRightFrontMotor.setStatorCurrent(DriveConstants.STATOR_CURRENT_LIMIT);
      simLeftBackMotor.setStatorCurrent(DriveConstants.STATOR_CURRENT_LIMIT);
      simRightBackMotor.setStatorCurrent(DriveConstants.STATOR_CURRENT_LIMIT);


      // The encoder and gyro angle sims let us set simulated sensor readings
      int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
      gyroAngle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));


      // the Field2d class lets us visualize our robot in the simulation GUI.
      fieldSim = new Field2d();
      SmartDashboard.putData("Field", fieldSim);
    } else {
      fieldSim = null;
      gyroAngle = null;
      simLeftBackMotor = null;
      simLeftFrontMotor = null;
      simRightBackMotor = null;
      simRightFrontMotor = null;
    }
  }

  @Override
  public void simulationPeriodic() {
    // To update our simulation, we set motor voltage inputs, update the simulation,
    // and write the simulated positions and velocities to our simulated encoder and gyro.
    // We negate the right side so that positive voltages make the right side
    // move forward.
    drivetrainSimulator.setInputs(
        leftMotors.get() * RobotController.getBatteryVoltage(),
        rightMotors.get() * RobotController.getBatteryVoltage());
    drivetrainSimulator.update(0.020);
    // System.out.println(RobotController.getBatteryVoltage());
    // System.out.println(getLeftEncoderDistanceMeters());
    // System.out.println(getRightEncoderDistanceMeters());
    simLeftFrontMotor.setIntegratedSensorRawPosition(
        (int) LobstahMath.distanceToNativeUnits(drivetrainSimulator.getLeftPositionMeters()));
    // System.out.println(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
    // System.out.println(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
    simLeftBackMotor.setIntegratedSensorRawPosition(
        (int) LobstahMath.distanceToNativeUnits(drivetrainSimulator.getLeftPositionMeters()));
    simRightFrontMotor.setIntegratedSensorRawPosition(
        -(int) LobstahMath.distanceToNativeUnits(drivetrainSimulator.getRightPositionMeters()));
    simRightBackMotor.setIntegratedSensorRawPosition(
        -(int) LobstahMath.distanceToNativeUnits(drivetrainSimulator.getRightPositionMeters()));
    simLeftFrontMotor.setIntegratedSensorVelocity((int) LobstahMath.metersPerSecondToFalcon500Velocity(
        drivetrainSimulator.getLeftVelocityMetersPerSecond(), Units.inchesToMeters(3)));
    simLeftBackMotor.setIntegratedSensorVelocity((int) LobstahMath.metersPerSecondToFalcon500Velocity(
        drivetrainSimulator.getLeftVelocityMetersPerSecond(), Units.inchesToMeters(3)));
    simRightFrontMotor.setIntegratedSensorVelocity(
        -(int) LobstahMath.metersPerSecondToFalcon500Velocity(drivetrainSimulator.getRightVelocityMetersPerSecond(),
            Units.inchesToMeters(3)));
    simRightBackMotor.setIntegratedSensorVelocity(
        -(int) LobstahMath.metersPerSecondToFalcon500Velocity(drivetrainSimulator.getRightVelocityMetersPerSecond(),
            Units.inchesToMeters(3)));
    gyroAngle.set(-drivetrainSimulator.getHeading().getDegrees());
    // gyroAngle.set(getGyroAngle180().getDegrees());

    poseEstimator.update(Rotation2d.fromDegrees(gyroAngle.get()), getLeftEncoderDistanceMeters(),
        getRightEncoderDistanceMeters());


    SmartDashboard.putNumber("Gyro", this.getGyroAngle180().getDegrees());
    SmartDashboard.putData("Field", fieldSim);
    SmartDashboard.putNumber("Number of Tags Visible In Front", this.photonVision.getFrontTargets().size());
    SmartDashboard.putNumber("Number of Tags Visible In Rear", this.photonVision.getRearTargets().size());
  }

  /**
   * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want it to work elsewhere,
   * use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
   *
   * @return The drawn current in Amps.
   */
  public double getDrawnCurrentAmps() {
    return drivetrainSimulator.getCurrentDrawAmps();
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
    poseEstimator.update(getGyroAngle180(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
    EstimatedRobotPose estimatedVisionPose = this.photonVision.getCurrentPose();
    if (estimatedVisionPose != null) {
      if (estimatedVisionPose.targetArea > VisionConstants.MIN_TARGET_AREA) {
        if (!hasSeenTag) {
          setGyroOffset(estimatedVisionPose.estimatedPose.getRotation());
          hasSeenTag = true;
        }
        poseEstimator.addVisionMeasurement(estimatedVisionPose.estimatedPose,
            estimatedVisionPose.timestampSeconds);
      }
    }
    return poseEstimator.getEstimatedPosition();
  }

  /**
   * Flips the given {@Pose2d} waypoint based on alliance color.
   *
   * @param waypoint The waypoint to flip
   * @param flipRotation Whether or not to flip the pose rotation.
   */
  public Pose2d flipWaypointBasedOnAlliance(Pose2d waypoint, boolean flipRotation) {
    if (DriverStation.getAlliance() == Alliance.Red) {
      if (flipRotation) {
        return new Pose2d(16.5 - waypoint.getX(), waypoint.getY(),
            waypoint.getRotation().plus(Rotation2d.fromDegrees(180)));
      } else {
        return new Pose2d(16.5 - waypoint.getX(), waypoint.getY(),
            waypoint.getRotation());
      }
    }
    return waypoint;
  }

  /**
   * Flips the given {@Pose2d} waypoint based on alliance color.
   *
   * @param waypointSupplier A supplier for the waypoint to flip
   * @param flipRotation Whether or not to flip the pose rotation.
   */
  public Pose2d flipWaypointBasedOnAlliance(Supplier<Pose2d> waypointSupplier, boolean flipRotation) {
    Pose2d waypoint = waypointSupplier.get();
    if (DriverStation.getAlliance() == Alliance.Red) {
      if (flipRotation) {
        return new Pose2d(16.5 - waypoint.getX(), waypoint.getY(),
            Rotation2d.fromDegrees(MathUtil
                .inputModulus(waypoint.getRotation().plus(Rotation2d.fromDegrees(180)).getDegrees(), -180, 180)));
      } else {
        return new Pose2d(16.5 - waypoint.getX(), waypoint.getY(),
            waypoint.getRotation());
      }
    }
    return waypoint;
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
    zeroGyro();
    setGyroOffset(rotation);
    resetEncoders();
    poseEstimator.resetPosition(getGyroAngle180(), 0, 0, new Pose2d(translation2d, rotation));
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
    return LobstahMath.nativeUnitsToDistanceMeters(leftFrontMotor.getSelectedSensorPosition());
  }

  /**
   * Gets the distance of the right encoder in meters.
   */
  public double getRightEncoderDistanceMeters() {
    return LobstahMath.nativeUnitsToDistanceMeters(rightBackMotor.getSelectedSensorPosition());
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
    gyro.setAngleAdjustment(0);
    gyro.reset();
  }

  /** Initializes the robot odometry based on Photonvision pose if available, or else uses assumed starting position. */
  public void initOdometry(Pose2d defaultPose) {
    EstimatedRobotPose estimatedVisionPose = photonVision.getCurrentPose();
    if (estimatedVisionPose != null) {
      zeroGyro();
      setGyroOffset(estimatedVisionPose.estimatedPose.getRotation());
      poseEstimator.resetPosition(getGyroAngle180(), 0, 0,
          estimatedVisionPose.estimatedPose);
      hasSeenTag = true;
      resetEncoders();
    } else {
      zeroGyro();
      setGyroOffset(defaultPose.getRotation());
      poseEstimator.resetPosition(getGyroAngle180(), 0, 0, defaultPose);
      hasSeenTag = false;
      System.out.println("Reset pose");
    }

    drivetrainSimulator.setPose(defaultPose);
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
   * Returns the robot's total accumulated/continuous angle as reported by the gyro.
   *
   * @return the robot's angle as a Rotation2d.
   */
  public Rotation2d getGyroAngle() {
    return gyro.getRotation2d();
  }

  /**
   * Returns the robot's angle as reported by the gyro, clamped between -180 and 180 degrees.
   *
   * @return the robot's angle from -180 to 180 as a Rotation2d.
   */
  public Rotation2d getGyroAngle180() {
    return Rotation2d.fromRadians(MathUtil.angleModulus(getGyroAngle().getRadians()));
  }

  /**
   * Set an amount with which to offset the value returned by {@link #getGyroAngle()}
   */
  public void setGyroOffset(Rotation2d offset) {
    gyro.setAngleAdjustment(offset.getDegrees());
  }

  /**
   * Returns the currently configured gyro offset.
   * 
   * @see {@link #setGyroOffset()}
   */
  public Rotation2d getGyroOffset() {
    return Rotation2d.fromDegrees(gyro.getAngleAdjustment());
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
   * Returns the distance in meters from the initial pose to the target Pose.
   */
  public double getDistanceBetweenPoses(Pose2d initialPose, Pose2d targetPose) {
    return initialPose.getTranslation().getDistance(targetPose.getTranslation());
  }

  /**
   * Returns the distance in meters from the pose of the robot to the target Pose.
   */
  public double getDirectDistanceToPose(Pose2d targetPose) {
    return getDistanceBetweenPoses(getPose(), targetPose);
  }

  /**
   * Returns the distance in meters from the pose of the robot to the target Pose.
   */
  public Transform2d getDistanceToPose(Pose2d targetPose) {
    return getPose().minus(targetPose);
  }

  /**
   * Generates a trajectory through a list of provided waypoints from the robot's position, assumed driving forwards.
   * 
   * @return A PathPlannerTrajectory to follow to the target position.
   */
  public PathPlannerTrajectory generatePath(List<Pose2d> waypoints) {
    return generatePath(false, waypoints);
  }

  /**
   * Generates a trajectory through a list of provided waypoints from the robot's position, assumed driving forwards.
   * 
   * @return A PathPlannerTrajectory to follow to the target position.
   */
  public PathPlannerTrajectory generatePath(Pose2d... waypoints) {
    return generatePath(false, waypoints);
  }

  /**
   * Generates a trajectory through a list of provided waypoints from the robot's position.
   * 
   * @param isReversed Whether the trajectory should be driven backwards.
   * @param waypoints A list of waypoints to generate a trajectory through.
   * 
   * @return A PathPlannerTrajectory to follow to the target position.
   */
  public PathPlannerTrajectory generatePath(boolean isReversed, List<Pose2d> waypoints) {
    return generatePath(isReversed, PathConstants.MAX_DRIVE_SPEED, PathConstants.MAX_ACCELERATION,
        waypoints);
  }

  /**
   * Generates a trajectory through a list of provided waypoints from the robot's position.
   * 
   * @param isReversed Whether the trajectory should be driven backwards.
   * @param waypoints The waypoints to generate a trajectory through.
   * 
   * @return A PathPlannerTrajectory to follow to the target position.
   */
  public PathPlannerTrajectory generatePath(boolean isReversed, Pose2d... waypoints) {
    return generatePath(isReversed, PathConstants.MAX_DRIVE_SPEED, PathConstants.MAX_ACCELERATION,
        waypoints);
  }

  /**
   * Generates a trajectory through a list of provided waypoints from the robot's position.
   * 
   * @param isReversed Whether the trajectory should be driven backwards.
   * @param maxDriveSpeed The maximum drive speed following the trajectory
   * @param maxAcceleration The maximum acceleration following the trajectory
   * @param waypoints A list of waypoints to generate a trajectory through.
   * 
   * @return A PathPlannerTrajectory to follow to the target position.
   */
  public PathPlannerTrajectory generatePath(boolean isReversed, double maxDriveSpeed, double maxAcceleration,
      List<Pose2d> waypoints) {
    ArrayList<PathPoint> pathPoints = new ArrayList<>();
    pathPoints.add(new PathPoint(this.getPose().getTranslation(), this.getPose().getRotation()));
    for (Pose2d waypoint : waypoints) {
      pathPoints.add(new PathPoint(waypoint.getTranslation(), waypoint.getRotation()));
    }
    return PathPlanner
        .generatePath(new PathConstraints(maxDriveSpeed, maxAcceleration), isReversed,
            pathPoints);
  }

  /**
   * Generates a trajectory through the provided waypoints from the robot's position to the target pose.
   * 
   * @param isReversed Whether the trajectory should be driven backwards.
   * @param maxDriveSpeed The maximum drive speed following the trajectory
   * @param maxAcceleration The maximum acceleration following the trajectory
   * @param waypoints The waypoints to generate a trajectory through.
   * 
   * @return A PathPlannerTrajectory to follow to the target position.
   */
  public PathPlannerTrajectory generatePath(boolean isReversed, double maxDriveSpeed, double maxAcceleration,
      Pose2d... waypoints) {
    ArrayList<PathPoint> pathPoints = new ArrayList<>();
    pathPoints.add(new PathPoint(this.getPose().getTranslation(), this.getPose().getRotation()));
    for (Pose2d pose : waypoints) {
      pathPoints.add(new PathPoint(pose.getTranslation(), pose.getRotation()));
    }
    return PathPlanner
        .generatePath(new PathConstraints(maxDriveSpeed, maxAcceleration),
            isReversed,
            pathPoints);
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
    poseEstimator.update(getGyroAngle180(), getLeftEncoderDistanceMeters(), getRightEncoderDistanceMeters());
    fieldSim.setRobotPose(getPose());
    EstimatedRobotPose estimatedVisionPose = this.photonVision.getCurrentPose();
    SmartDashboard.putString("PhotonVision Pose",
        estimatedVisionPose == null ? "Null" : estimatedVisionPose.estimatedPose.toString());
    if (estimatedVisionPose != null) {
      if (estimatedVisionPose.targetArea > VisionConstants.MIN_TARGET_AREA) {
        if (!hasSeenTag) {
          setGyroOffset(estimatedVisionPose.estimatedPose.getRotation());
          hasSeenTag = true;
        }
        poseEstimator.addVisionMeasurement(estimatedVisionPose.estimatedPose,
            estimatedVisionPose.timestampSeconds);
      }
    }

    // if (!gyro.isCalibrating() && !gyroInitialized) {
    // if (gyro.getVelocityZ() != 0) {
    // gyroInitialized = true;
    // }
    // }

    SmartDashboard.putBoolean("Gyro Initialized", gyroInitialized);
    SmartDashboard.putNumber("Gyro", this.getGyroAngle180().getDegrees());
    SmartDashboard.putNumber("Gyro 180", this.getGyroAngle180().getDegrees());
    SmartDashboard.putNumber("Gyro Offset", getGyroOffset().getDegrees());
    SmartDashboard.putString("Pose", this.getPose().toString());
    SmartDashboard.putNumber("Number of Tags Visible In Front", this.photonVision.getFrontTargets().size());
    SmartDashboard.putNumber("Number of Tags Visible In Rear", this.photonVision.getRearTargets().size());
    SmartDashboard.putBoolean("Has seen tag", hasSeenTag);
  }
}
