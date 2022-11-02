// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
   * Stores constants related to autonomous routines.
   */
  public static final class AutonConstants {
    public static final double SIMPLE_AUTON_SPEED = 0.7;
    public static final double SIMPLE_AUTON_RUNTIME = 3.0;
    public static final double MEDIUM_AUTON_OUTTAKE_RUNTIME = 3.0;

    public static final class ComplexAutonConstants {
      public static final double COMPLEX_AUTON_MAX_SPEED = 0.7;
      public static final double COMPLEX_AUTON_MAX_ACCELERATION = 0.5;
      public static final double RAMSETE_B = 2.0;
      public static final double RAMSETE_ZETA = 0.7;
      public static final double KS = 0.0;
      public static final double KV = 0.0;
      public static final double KA = 0.0;
      public static final double KP = 0.0;

      public static final double TRACK_WIDTH = 27.0;
      public static final DifferentialDriveKinematics KINEMATICS =
          new DifferentialDriveKinematics(Units.inchesToMeters(TRACK_WIDTH));
    }
  }

  /**
   * Stores constants related to driver controls, SmartDashboard and other IO (Input/Output).
   */
  public static final class IOConstants {
    public static final int DRIVER_JOYSTICK_INDEX = 0;
    public static final int OPERATOR_JOYSTICK_INDEX = 1;

    public static final class DriverButtons {

    }

    public static final class DriverAxes {
      public static final int LEFT = 1;
      public static final int RIGHT = 5;
    }

    public static final class OperatorButtons {

    }
  }

  /**
   * Stores constants related to the DriveBase.
   */
  public static final class DriveConstants {
    public static final double ACCELERATION_RATE_LIMIT = 1;
    public static final int CURRENT_LIMIT = 50;
    public static final int TRIGGER_THRESHOLD = 100;
    public static final double TRIGGER_THRESHOLD_TIME = 1;

    public static final class DriveMotorCANIDs {
      public static final int RIGHT_FRONT = 0;
      public static final int RIGHT_BACK = 1;
      public static final int LEFT_FRONT = 2;
      public static final int LEFT_BACK = 3;
    }
  }
}
