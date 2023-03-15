// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
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
   * Stores constants related to path following.
   */
  public static final class PathConstants {
    public static final double MAX_DRIVE_SPEED = 0.6;
    public static final double MAX_ACCELERATION = 0.4;
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double kS = 0.032478;
    public static final double kV = 5.3084;
    public static final double kA = 1.9328;
    public static final double kP = 1.2264;
    public static final double kI = 0;
    public static final double KD = 0;
    public static final double TURN_P = 0.035;
    public static final double TURN_I = 0;
    public static final double TURN_D = 0.02;
    public static final double TURN_ANGLE_DEADBAND = 2;
    public static final double POSE_DISTANCE_METERS_FILTER = 10;
  }

  /**
   * Stores constants related to this year's playing field.
   */
  public static final class FieldConstants {
    public static final double SCORING_WAYPOINTS_X = 1.8;
    public static final double TRAVELING_WAYPOINTS_X = 2.2;

    public static final Pose2d[] SCORING_WAYPOINTS = new Pose2d[] {
        new Pose2d(SCORING_WAYPOINTS_X, 0.5, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 1, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 1.6, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 2.2, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 2.7, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 3.3, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 3.85, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 4.4, Rotation2d.fromDegrees(180)),
        new Pose2d(SCORING_WAYPOINTS_X, 5, Rotation2d.fromDegrees(180))
    };
    public static final Pose2d[] TRAVELING_WAYPOINTS = new Pose2d[] {
        new Pose2d(TRAVELING_WAYPOINTS_X, 0.5, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 0.75, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 1, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 1.6, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 2.2, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 2.7, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 3.3, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 3.85, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 4.35, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 4.7, new Rotation2d(0)),
        new Pose2d(TRAVELING_WAYPOINTS_X, 5, new Rotation2d(0))
    };

    // public static final Pose2d[] TURNING_WAYPOINTS = new Pose2d[] {
    // new Pose2d(2.5, 0.77, new Rotation2d(0)), new Pose2d(2.70, 5, new Rotation2d(0))
    // };

    public static final Pose2d[] CROSSING_WAYPOINTS = new Pose2d[] {
        new Pose2d(5.85, 0.6, new Rotation2d(0)), new Pose2d(5.85, 4.85, new Rotation2d(0))
    };

    public static final Pose2d[] RETURNING_CROSSING_WAYPOINTS = new Pose2d[] {
        new Pose2d(5.50, 0.75, new Rotation2d(90)), new Pose2d(4.75, 5, new Rotation2d(-90))
    };

    public static final Pose2d[] ENTERING_SCORING_ZONE_WAYPOINTS = new Pose2d[] {
        new Pose2d(2.95, 0.75, new Rotation2d(180)), new Pose2d(2.95, 5, new Rotation2d(180))
    };

    // public static final Pose2d[] ENDING_AUTON_POSES = new Pose2d[] {
    // new Pose2d(6.00, 0.93, new Rotation2d(0)), new Pose2d(6.00, 2.15, new Rotation2d(0)),
    // new Pose2d(6.50, 6.00, new Rotation2d(0)), new Pose2d(7.60, 7.26, new Rotation2d(0))
    // };

    public static final Pose2d[] GROUND_PICKUP_POSES = new Pose2d[] {
        new Pose2d(7.40, 0.93, new Rotation2d(0)), new Pose2d(7.40, 2.15, new Rotation2d(0)),
        new Pose2d(7.40, 3.37, new Rotation2d(0)), new Pose2d(7.40, 4.59, new Rotation2d(0))
    };

    public static final Pose2d PLAYER_STATION_PICKUP_LEFT = new Pose2d(15.75, 7.452, new Rotation2d(0));
    public static final Pose2d PLAYER_STATION_PICKUP_RIGHT = new Pose2d(15.75, 6.148, new Rotation2d(0));

    public static final double MAX_AUTO_DISTANCE_METERS = 10;
    public static final double MAX_PLAYER_STATION_X_ZONE = 7;
    public static final double MAX_PLAYER_STATION_Y_ZONE = 5.5;
    public static final double SCORING_ZONE_DEADBAND = 0.5;
    public static final double SCORING_ZONE_X = 3.5;
    public static final double FIELD_LENGTH = 16.5;
  }

  /**
   * Stores constants related to autonomous routines.
   */
  public static final class AutonConstants {
    public static final double SIMPLE_AUTON_SPEED = 0.3;
    public static final double SIMPLE_AUTON_RUNTIME = 3.25;
    public static final double AUTON_SCORING_TOLERANCE = 2;
    public static final double OUTTAKE_RUNTIME = 1;
    public static final double BACK_OFF_SPEED = -0.1;
    public static final double DRIVE_BACK_SPEED = -0.3;
    public static final double DRIVE_BACK_TIME = 0.5;
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
        new Transform3d(new Translation3d(-Units.inchesToMeters(12.1585), 0.07, Units.inchesToMeters(20.749)),
            new Rotation3d(-Math.PI / 2, 0, Math.PI));
    public static final double MAINTAIN_CAMERA_CONFIDENCE_THRESHOLD = 0.7;
    public static final double MIN_TARGET_AREA = 0.3;
  }

  /**
   * Stores constants related to driver controls, SmartDashboard and other operator interface elements.
   */
  public static final class OIConstants {
    public static final class DriverConstants {
      public static final int DRIVER_USB_INDEX = 0;
      public static final boolean SQUARED_INPUTS = true;
      public static final int LEFT_AXIS = 1;
      public static final int RIGHT_AXIS = 5;
      public static final int SLOWDOWN_BTN = 5;
      public static final double SLOWDOWN_FACTOR = 0.5;
      public static final int TARGET_BTN = 1;
    }

    public static final class OperatorConstants {
      public static final int OPERATOR_USB_INDEX = 1;
      public static final double JOYSTICK_DEADBAND = 0.05;

      public static final int INTAKE_BTN = 5;
      public static final int OUTTAKE_BTN = 6;

      public static final int MANUAL_CONTROL_BTN = 8;
      public static final double MANUAL_CONTROL_SPEED = 1.3;
      public static final int MANUAL_X_JOYSTICK_AXIS = 1;
      public static final int MANUAL_Y_JOYSTICK_AXIS = 5;

      // public static final int LEFT_PICKUP_POV = 270;
      // public static final int RIGHT_PICKUP_POV = 90;
      // public static final int GROUND_PICKUP_POV = 180;

      public static final int LOW_GOAL_POV = 180;
      public static final int MID_GOAL_POV = 90;
      public static final int HIGH_GOAL_POV = 0;
      public static final int PLAYER_STATION_POV = 270;

      public static final int LEFT_PICKUP_BTN = 1;
      public static final int RIGHT_PICKUP_BTN = 3;
      public static final int GROUND_PICKUP_BTN = 2;

      // public static final int LOW_GOAL_BTN = 2;
      // public static final int MID_GOAL_BTN = 3;
      // public static final int HIGH_GOAL_BTN = 4;
      // public static final int PLACE_DOWN_BTN = 1;
    }
  }

  /**
   * Stores constants related the arm (pivot and elevator).
   */
  public static final class ArmConstants {
    public static final Translation2d ROBOT_TO_SCORING_ORIGIN = new Translation2d(0, 0); // TODO
    public static final double RETRACT_BEFORE_ROTATING_ANGLE = 5;
    public static final double RETRACT_BEFORE_ROTATING_PRECISION = 5;
    public static final Rotation2d BUMPER_AVOIDANCE_ANGLE = Rotation2d.fromDegrees(52);
    public static final double BUMPER_AVOIDANCE_X = 0;
    public static final double BUMPER_AVOIDANCE_EXTENSION_PRECISION = 0.5;
    public static final double BUMPER_AVOIDANCE_ANGLE_PRECISION = 3;
    public static final double SWITCH_TO_PARALLEL_ANGLE_PRECISION = 7;
    public static final double MIN_Y_POSITION = 6;

    /**
     * Stores constants related to pivot.
     */
    public static final class PivotConstants {
      // Geometry
      public static final Translation2d ORIGIN_TO_PIVOT = new Translation2d(-24.343, 50);
      public static final double PIVOT_OFFSET_DEG = 319;

      // IO
      public static final int ENCODER_CHANNEL = 3;
      public static final double PIVOT_DEGREES_PER_ROTATION = 360;
      public static final int LEFT_MOTOR_ID = 21; // TODO: figure out which is left and which is right
      public static final int RIGHT_MOTOR_ID = 22;

      // Limits
      public static final int CURRENT_LIMIT = 40;
      public static final double MAX_VELOCITY_DEG_PER_SEC = 300;
      public static final double MAX_ACCELERATION_DEG_PER_SEC_SQUARED = 400;
      public static final double MAX_ROTATION_DEG = 105;
      public static final double MIN_ROTATION_DEG = 35;

      // PID
      public static final double P = 0.03;
      public static final double S_VOLTS = 0;
      public static final double G_VOLTS = 0;
      public static final double A_VOLT_SECOND_SQUARED_PER_RAD = 0;
      public static final double V_VOLT_SECOND_PER_RAD = 0;
      public static final double ROTATION_PID_TOLERANCE = 1;
    }

    /**
     * Stores constants related to the elevator.
     */
    public static final class ElevatorConstants {
      public static final double LENGTH_FULLY_RETRACTED = 38;
      public static final double HOME_SPEED = 0.65;

      // IO
      public static final int ENCODER_CHANNEL_A = 0;
      public static final int ENCODER_CHANNEL_B = 1;
      public static final double DISTANCE_PER_PULSE = 5.5 / 2048; // 5.5 inches for a 22 tooth-sprocket with 1/4" chain
                                                                  // links
      public static final int LIMIT_SWITCH_CHANNEL = 2;
      public static final int ELEVATOR_MOTOR_ID = 31;

      // Limits
      public static final int CURRENT_LIMIT = 20;
      public static final double MAX_EXTENSION_INCHES = 29;
      public static final double MIN_EXTENSION_INCHES = -0.1;
      public static final double MAX_VELOCITY_INCHES_PER_SEC = 10;
      public static final double MAX_ACCELERATION_INCHES_PER_SEC_SQUARED = 5;

      // PID
      public static final double S = 0.32321;
      public static final double V = 0.123766;
      public static final double A = 0.0853;
      public static final double G = -0.11681;
      public static final double P = 0.16023;
    }
  }

  /**
   * Stores constants related to the intake.
   */
  public static final class IntakeConstants {
    public static final Translation2d INTAKE_OFFSET = new Translation2d(4.281, 6.37);

    // IO
    public static final int LEFT_MOTOR_ID = 0;
    public static final int RIGHT_MOTOR_ID = 1;

    // Speeds
    public static final double OUTTAKE_VOLTAGE = 1.8;
    public static final double INTAKE_VOLTAGE = -5;
    public static final double PASSIVE_INTAKE_VOLTAGE = -1.6;
  }

  /**
   * Stores preset poses for the arm.
   */
  public static final class ArmPresets {
    public static final ArmPose STOWED = ArmPose.fromAngleExtension(Rotation2d.fromDegrees(36), -0.1);
    public static final ArmPose GROUND_PICKUP = ArmPose.fromXY(16.4, 10.2);
    public static final ArmPose MID_GOAL_SCORING = ArmPose.fromXY(25, 47); // before dropping
    public static final ArmPose LOW_GOAL_SCORING = ArmPose.fromXY(15, 14);
    public static final ArmPose HIGH_GOAL_SCORING = ArmPose.fromXY(42, 59);
    public static final Translation2d CONE_SCORING_DROPDOWN = new Translation2d(0, -10);
    public static final Translation2d CONE_SCORING_BACKOFF = new Translation2d(-4, -7);
    public static final Translation2d CONE_SCORING_BACK_UP = new Translation2d(-4, 0);
    public static final ArmPose PLAYER_STATION_PICKUP = ArmPose.fromXY(18, 46);
  }


  /**
   * Stores constants related to the DriveBase.
   */
  public static final class DriveConstants {
    public static final int COUNTS_PER_REV = 2048;
    public static final double SENSOR_GEAR_RATIO = 8.458646;
    public static final double WHEEL_RADIUS_INCHES = 2.97262;
    public static final double TRACK_WIDTH = 18.75;

    public static final class DriveMotorCANIDs {
      public static final int RIGHT_FRONT = 14;
      public static final int RIGHT_BACK = 13;
      public static final int LEFT_FRONT = 12;
      public static final int LEFT_BACK = 11;
    }

    public static final int STATOR_CURRENT_LIMIT = 80;
    public static final int SUPPLY_CURRENT_LIMIT = 80;
    public static final int STATOR_TRIGGER_THRESHOLD = 100;
    public static final double STATOR_TRIGGER_THRESHOLD_TIME = 0.5;
    public static final int SUPPLY_TRIGGER_THRESHOLD = 100;
    public static final double SUPPLY_TRIGGER_THRESHOLD_TIME = 0.5;
    public static final double SLEW_RATE_LIMIT = 1.8;

    public static final double ACCELERATION_RATE_LIMIT = 1.5;
    public static final DifferentialDriveKinematics KINEMATICS =
        new DifferentialDriveKinematics(Units.inchesToMeters(TRACK_WIDTH));
  }
}
