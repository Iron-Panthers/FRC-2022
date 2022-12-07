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
                ? -Math.toRadians(224.648) // comp bot offset
                : -Math.toRadians(39.462890); // practice bot offset
      }

      public static final class FrontLeft { // Module 2
        public static final int DRIVE_MOTOR = 11;
        public static final int STEER_MOTOR = 10;
        public static final int STEER_ENCODER = 25;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(331.43) // comp bot offset
                : -Math.toRadians(222.7148); // practice bot offset
      }

      public static final class BackLeft { // Module 3
        public static final int DRIVE_MOTOR = 13;
        public static final int STEER_MOTOR = 12;
        public static final int STEER_ENCODER = 26;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(7.734) // comp bot offset
                : -Math.toRadians(129.63867); // practice bot offset
      }

      public static final class BackRight { // Module 4
        public static final int DRIVE_MOTOR = 2;
        public static final int STEER_MOTOR = 1;
        public static final int STEER_ENCODER = 27;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(19.863) // comp bot offset
                : -Math.toRadians(61.3476); // practice bot offset
      }
    }
  }

  public static final class Arm {
    // Throw any Arm constants in this file
    public static final class Ports {
      public static final int RIGHT_MOTOR_PORT = 14;
      public static final int LEFT_MOTOR_PORT = 5;
      public static final int ENCODER_PORT = 22;
    }

    public static final double ANGULAR_OFFSET = IS_COMP_BOT ? 58 : 60;
    public static final double GRAVITY_CONTROL_PERCENT = .06;

    public static final class Hardstop {
      public static final double HOLD_PERCENT = .07;
      public static final double ERROR_MARGIN = 15;
    }

    public static final class PID {
      public static final double ANGULAR_TOLERANCE = 1.0;
    }

    public static final class Setpoints {
      public static final double MAX_HEIGHT = 145.811;

      public static final double OUTTAKE_HIGH_POSITION = MAX_HEIGHT;
      public static final double INTAKE_POSITION = MAX_HEIGHT - 155.3;
      public static final double INTAKE_HIGHER_POSITION = MAX_HEIGHT - 93.3;
      public static final double CLIMB_POSITION = MAX_HEIGHT - 37.2;
    }
  }

  public static final class Elevator {

    public static final double POSITION = 1.0;
    public static final double RATE = 0.025;

    public static final double MAX_PERCENT = .7;

    // Heights
    /** Max height is 21.75 inches (adjusted for overshoot) */
    public static final double maxHeight = 20;

    /** Minimum height is 0 inches */
    public static final double minHeight = 0;

    /** the minimum height the arm will stay out of the way for, inches */
    public static final double ENGAGED_HEIGHT = 1.2;

    /** run the motors at lower power when height is higher then upper or lower then lower. */
    public static final class SlowZone {
      public static final double SLOWZONE_MODIFIER = .25;
      public static final double UPPER_THRESHHOLD = 16;
      public static final double LOWER_THRESHHOLD = 7.5;
    }

    public static final double TOP_LIMIT_SWITCH_TRIGGER_HEIGHT = -19.5;
    public static final double BOTTOM_LIMIT_SWITCH_TRIGGER_HEIGHT = -.5;

    public static final int TICKS = 2048;
    public static final double GEAR_RATIO = 12.75;
    public static final double GEAR_CIRCUMFERENCE = 1.5 * Math.PI;

    public static final class Ports {
      public static final int LEFT_MOTOR = 7;
      public static final int RIGHT_MOTOR = 6;
      public static final int BOTTOM_SWITCH = 9;
      public static final int TOP_SWITCH = 0;
    }
  }

  public static final class Intake {

    /** time in seconds to wait between certain modes */
    public static final class ModeWaits {
      /** the high goal shot, over the elevator */
      public static final class High {
        public static final double CENTER_NORMALIZE_TO_ALIGN = 0.1;
        public static final double ALIGN_TO_LEFT = .2;
        public static final double LEFT_TO_ALL = .7;
        public static final double ALL_TO_OFF = 1.5;
      }

      /** this is out taking backwards, ie over the elevator (LOW SHOT) */
      public static final class Outtake {
        public static final double ALIGN_TO_LEFT = .24 * 2;
        public static final double LEFT_TO_ALL = .55 * 3;
        public static final double ALL_TO_OFF = 2;
      }

      /** time between states for intake sequence */
      public static final class IntakeWaits {
        public static final double IDLE_TO_OFF = 1;
      }
    }

    /** percent to run motors at for given states */
    public static final class EjectRollers {
      /** the percent to run the eject motor at in the idling state */
      public static final double IDLE = 1;
      /** the percent to run the eject motor at in the ejection state */
      public static final double EJECT = -1;

      /** the percent to run for high shot */
      public static final double FEED_HIGH = .1;

      /** the percent to run for the low shot */
      public static final double FEED_LOW = .7;

      public static final double ALIGN_INTERNAL = -0.5;
    }

    /** percent to run motors at for given states */
    public static final class IntakeRollers {
      /** the percent to run the intake motors during the intake state */
      public static final double INTAKE = -.35;
      /** speed to run intake motors for unsticking balls */
      public static final double INTAKE_FORCEFUL = -1;
      /** LOW SHOT; the velocity to run the upper outtake during the outtake state */
      public static final double OUTTAKE_UPPER_LOW = 8000;
      /** LOW SHOT; the velocity to run the lower outtake during the outtake state */
      public static final double OUTTAKE_LOWER_LOW = 8000;
      /** percent */
      public static final double ALIGN_INTERNAL = -0.1103 * .5;

      // velocity speeds for the high shoot
      /** HIGH SHOT; velocity */
      public static final double OUTTAKE_UPPER_HIGH = 12_000;
      /** HIGH SHOT; velocity */
      public static final double OUTTAKE_LOWER_HIGH = 10_000;
    }

    public static final class Ports {
      public static final int LOWER_MOTOR = 15;
      public static final int UPPER_MOTOR = 9;
      public static final int RIGHT_EJECT_MOTOR = 16;
      public static final int LEFT_EJECT_MOTOR = 8;
    }
  }
}
