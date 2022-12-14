// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Transform3d;
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
   * Stores constants related to autonomous routines.
   */
  public static final class AutonConstants {
    public static final double SIMPLE_AUTON_SPEED = 0.7;
    public static final double SIMPLE_AUTON_RUNTIME = 3.0;
  }

  public static final class VisionConstants {
    public static final Transform3d CAMERA_TO_ROBOT = new Transform3d();
  }

  /**
   * Stores constants related to driver controls, SmartDashboard and other user interface elements.
   */
  public static final class UIConstants {
    public static final int DRIVER_JOYSTICK_INDEX = 0;
    public static final int OPERATOR_JOYSTICK_INDEX = 1;

    public static final boolean SQUARED_INPUTS = true;

    public static final class DriverAxes {
      public static final int LEFT = 1;
      public static final int RIGHT = 5;
    }
  }

  /**
   * Stores constants related to the DriveBase.
   */
  public static final class DriveConstants {

    public static final double ACCELERATION_RATE_LIMIT = 2.1;
    public static final DifferentialDriveKinematics KINEMATICS =
        new DifferentialDriveKinematics(Units.inchesToMeters(RobotConstants.TRACK_WIDTH));

    public static final class DriveMotorCANIDs {
      public static final int RIGHT_FRONT = 44;
      public static final int RIGHT_BACK = 43;
      public static final int LEFT_FRONT = 42;
      public static final int LEFT_BACK = 41;
    }

    public static final int CURRENT_LIMIT = 60;
    public static final int TRIGGER_THRESHOLD = 80;
    public static final double TRIGGER_THRESHOLD_TIME = 0.5;
  }
}
