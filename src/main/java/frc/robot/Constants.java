// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to
 * reduce verbosity.
 */
public final class Constants {

  /**
   * Stores constants related to the robot.
   */
  public static final class RobotConstants {
    public static final int kCountsPerRev = 2048; // Encoder counts per revolution of the motor shaft.
    public static final double kSensorGearRatio = 10.71; // Gear ratio is the ratio between the *encoder* and the
                                                         // wheels. On
    // the
    // AndyMark drivetrain, encoders mount 1:1 with the gearbox shaft.
    public static final double kGearRatio = 10.71; // Switch kSensorGearRatio to this gear ratio if encoder is on the
                                                   // motor
    // instead of on the gearbox.
    public static final double kWheelRadiusInches = 6;
    public static final int k100msPerSecond = 10;
    public static final double TRACK_WIDTH = 27.0;
  }

  /**
   * Stores constants related to path following.
   */
  public static final class PathConstants {
    public static final double MAX_DRIVE_SPEED = 1;
    public static final double MAX_ACCELERATION = 1;
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double KS = 0.56859;
    public static final double KV = 2.4414;
    public static final double KA = 0.24643;
    public static final double KP = 0.00000094597;
    public static final double KI = 0;
    public static final double KD = 0;
  }

  /**
   * Stores constants related to this year's playing field.
   */
  public static final class FieldConstants {
    public static final Pose2d[] SCORING_WAYPOINTS = new Pose2d[] {
        new Pose2d(1.57, 0.43, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 1, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 1.6, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 2.2, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 2.7, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 3.3, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 3.85, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 4.4, new Rotation2d(Math.toRadians(180))),
        new Pose2d(1.57, 5, new Rotation2d(Math.toRadians(180)))
    };
    public static final Pose2d[] TRAVELING_WAYPOINTS = new Pose2d[] {
        new Pose2d(2.75, 0.43, new Rotation2d(0)), new Pose2d(2.75, 1, new Rotation2d(0)),
        new Pose2d(2.75, 1.6, new Rotation2d(0)), new Pose2d(2.75, 2.2, new Rotation2d(0)),
        new Pose2d(2.75, 2.7, new Rotation2d(0)), new Pose2d(2.75, 3.3, new Rotation2d(0)),
        new Pose2d(2.75, 3.85, new Rotation2d(0)), new Pose2d(2.75, 4.4, new Rotation2d(0)),
        new Pose2d(2.75, 5, new Rotation2d(0))
    };
    public static final double MAX_AUTO_DISTANCE_METERS = 10;
    public static final double SCORING_ZONE_DEADBAND = 0.5;
  }

  /**
   * Stores constants related to autonomous routines.
   */
  public static final class AutonConstants {
    public static final double SIMPLE_AUTON_SPEED = 0.7;
    public static final double SIMPLE_AUTON_RUNTIME = 3.0;
  }

  public static final class VisionConstants {
    public static final Transform3d ROBOT_TO_FRONT_LEFT_CAMERA =
        new Transform3d(
            new Translation3d(-Units.inchesToMeters(9.766), Units.inchesToMeters(7.852), Units.inchesToMeters(28.242)),
            new Rotation3d());
    public static final Transform3d ROBOT_TO_FRONT_RIGHT_CAMERA =
        new Transform3d(
            new Translation3d(-Units.inchesToMeters(9.766), -Units.inchesToMeters(7.852), Units.inchesToMeters(28.242)),
            new Rotation3d());
    public static final Transform3d ROBOT_TO_REAR_CAMERA =
        new Transform3d(new Translation3d(-Units.inchesToMeters(12.1585), 0, Units.inchesToMeters(20.749)),
            new Rotation3d(0, 0, Math.PI));
    public static final double MAINTAIN_CAMERA_CONFIDENCE_THRESHOLD = 0.7;
  }

  /**
   * Stores constants related to driver controls, SmartDashboard and other user interface elements.
   */
  public static final class UIConstants {
    public static final int DRIVER_JOYSTICK_INDEX = 0;
    public static final int OPERATOR_JOYSTICK_INDEX = 1;

    public static final boolean SQUARED_INPUTS = true;

    public static final int SLOWDOWN_BUTTON_INDEX = 2;

    public static final class DriverAxes {
      public static final int LEFT = 1;
      public static final int RIGHT = 5;
    }
  }

  /**
   * Stores constants related to the DriveBase.
   */
  public static final class DriveConstants {

    public static final double ACCELERATION_RATE_LIMIT = 1.5;
    public static final DifferentialDriveKinematics KINEMATICS =
        new DifferentialDriveKinematics(Units.inchesToMeters(RobotConstants.TRACK_WIDTH));
    public static final double TURN_KP = 0.01;
    public static final double TURN_DEADBAND = 1;
    public static final double SLOWDOWN_PERCENT = 0.5;

    public static final class DriveMotorCANIDs {
      public static final int RIGHT_FRONT = 14;
      public static final int RIGHT_BACK = 13;
      public static final int LEFT_FRONT = 12;
      public static final int LEFT_BACK = 11;
    }

    public static final int STATOR_CURRENT_LIMIT = 50;
    public static final int SUPPLY_CURRENT_LIMIT = 80;
    public static final int STATOR_TRIGGER_THRESHOLD = 60;
    public static final double STATOR_TRIGGER_THRESHOLD_TIME = 0.1;
    public static final int SUPPLY_TRIGGER_THRESHOLD = 90;
    public static final double SUPPLY_TRIGGER_THRESHOLD_TIME = 0.5;
  }
}
