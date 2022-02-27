// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.util.MacUtil.IS_COMP_BOT;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

@SuppressWarnings("java:S1118")
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Drive {
    // max voltage delivered to drivebase
    // supposedly useful to limit speed for testing
    public static final double MAX_VOLTAGE = 12.0;
    // maximum velocity
    // FIXME measure this value experimentally
    public static final double MAX_VELOCITY_METERS_PER_SECOND =
        6380.0 // falcon 500 free speed rpm
            / 60.0
            * SdsModuleConfigurations.MK4_L2.getDriveReduction()
            * SdsModuleConfigurations.MK4_L2.getWheelDiameter()
            * Math.PI;
    // theoretical value
    // FIXME measure and validate experimentally
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        MAX_VELOCITY_METERS_PER_SECOND
            / Math.hypot(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0)
            * .5;

    /** the maximum amount of angular error pid loops will tolerate for rotation */
    public static final double ANGULAR_ERROR = 1.0;
    /** the minimum magnitude of the right stick for it to be used as a new rotation angle */
    public static final double ROTATE_VECTOR_MAGNITUDE = .7;

    public static final class Dims {
      // FIXME validate with hardware
      public static final double TRACKWIDTH_METERS =
          .5207; // 20.5 inches (source: cad) converted to meters
      public static final double WHEELBASE_METERS = TRACKWIDTH_METERS; // robot is square
    }

    public static final class Modules {
      public static final class FrontRight { // Module 1
        public static final int DRIVE_MOTOR = 4;
        public static final int STEER_MOTOR = 3;
        public static final int STEER_ENCODER = 24;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(329.4937 + 180.0) // comp bot offset
                : -Math.toRadians(40.333 + 180.0); // practice bot offset
      }

      public static final class FrontLeft { // Module 2
        public static final int DRIVE_MOTOR = 11;
        public static final int STEER_MOTOR = 10;
        public static final int STEER_ENCODER = 25;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(209.0836) // comp bot offset
                : -Math.toRadians(218.7512 + 180.0); // practice bot offset
      }

      public static final class BackLeft { // Module 3
        public static final int DRIVE_MOTOR = 13;
        public static final int STEER_MOTOR = 12;
        public static final int STEER_ENCODER = 26;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(183.5815) // comp bot offset
                : -Math.toRadians(188.6133); // practice bot offset
      }

      public static final class BackRight { // Module 4
        public static final int DRIVE_MOTOR = 2;
        public static final int STEER_MOTOR = 1;
        public static final int STEER_ENCODER = 27;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(20.9152 + 180.0) // comp bot offset
                : -Math.toRadians(329.6777); // practice bot offset
      }
    }
  }

  public static final class Arm {
    // Throw any Arm constants in this file
    public static final class Ports {
      public static final int RightMotorPort = 14;
      public static final int LeftMotorPort = 5;
      public static final int ENCODER_PORT = 22;
    }

    public static final double ANGULAR_OFFSET = 295.8;
    public static final double GRAVITY_CONTROL_PERCENT = .145;

    public static final class PID {
      public static final double ANGULAR_TOLERANCE = 1.0;
    }

    public static final class Setpoints {
      public static final double OUTTAKE_HIGH_POSITION = 75.53;
      public static final double OUTTAKE_LOW_POSITION = 48.5;
      public static final double INTAKE_POSITION = -26.15;
      public static final double MAX_HEIGHT = 81.07;
    }
  }

  public static final class Intake {
    /** the percent to run the idler motor at in the idling state */
    public static final double IDLER_PERCENT = 1;
    /** the percent to run the intake motors during the intake state */
    public static final double INTAKE_PERCENT = -.55;
    /** the percent to run the outtake motors during the outtake state */
    public static final double OUTTAKE_PERCENT = .7;
    /** the percent to run the idler motor at in the ejection state */
    public static final double EJECT_PERCENT = -.9;

    public static final class Ports {
      public static final int LOWER_MOTOR = 14;
      public static final int UPPER_MOTOR = 6;
      public static final int IDLER_MOTOR = 15;
    }
  }
}
