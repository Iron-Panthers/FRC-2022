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
                : -Math.toRadians(39.1937); // practice bot offset
      }

      public static final class FrontLeft { // Module 2
        public static final int DRIVE_MOTOR = 11;
        public static final int STEER_MOTOR = 10;
        public static final int STEER_ENCODER = 25;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(209.0836) // comp bot offset
                : -Math.toRadians(180 + 269.207); // practice bot offset
      }

      public static final class BackLeft { // Module 3
        public static final int DRIVE_MOTOR = 13;
        public static final int STEER_MOTOR = 12;
        public static final int STEER_ENCODER = 26;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(183.5815) // comp bot offset
                : -Math.toRadians(8.075); // practice bot offset
      }

      public static final class BackRight { // Module 4
        public static final int DRIVE_MOTOR = 2;
        public static final int STEER_MOTOR = 1;
        public static final int STEER_ENCODER = 27;

        public static final double STEER_OFFSET =
            IS_COMP_BOT
                ? -Math.toRadians(20.9152 + 180.0) // comp bot offset
                : -Math.toRadians(152.655); // practice bot offset
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

    public static final double ANGULAR_OFFSET = 58;
    public static final double GRAVITY_CONTROL_PERCENT = .135;

    public static final class PID {
      public static final double ANGULAR_TOLERANCE = 1.0;
    }

    public static final class Setpoints {
      public static final double OUTTAKE_HIGH_POSITION = 68;
      public static final double OUTTAKE_LOW_POSITION = 50;
      public static final double INTAKE_POSITION = -30;
      public static final double INTAKE_HIGHER_POSITION = -20;
      public static final double MAX_HEIGHT = 74.35;
    }
  }

  public static final class Elevator {

    public static final double POSITION = 1.0;
    public static final double RATE = 0.025;

    // Heights
    /** Max height is 21.75 inches (adjusted for overshoot) */
    public static final double maxHeight = 20;

    /** Minimum height is 0 inches */
    public static final double minHeight = 0;

    public static final int TICKS = 2048;
    public static final double GEAR_RATIO = 12.75;
    public static final double GEAR_CIRCUMFERENCE = 1.5 * Math.PI;

    public static final class Ports {
      public static final int LEFT_MOTOR = 6;
      public static final int RIGHT_MOTOR = 7;
      public static final int BOTTOM_SWITCH = 9;
      public static final int TOP_SWITCH = 0;
    }
  }

  public static final class Intake {
    /** percent to run motors at for given states */
    public static final class EjectRollers {
      /** the percent to run the eject motor at in the idling state */
      public static final double IDLE = 1;
      /** the percent to run the eject motor at in the ejection state */
      public static final double EJECT = -.9;
    }

    /** percent to run motors at for given states */
    public static final class IntakeRollers {
      /** the percent to run the intake motors during the intake state */
      public static final double INTAKE = .55;
      /** the percent to run the upper outtake during the outtake state */
      public static final double OUTTAKE_UPPER = -0.3;
      /** the percent to run the lower outtake during the outtake state */
      public static final double OUTTAKE_LOWER = -0.225;
      /**
       * the percent to run the upper outtake during the outtake fast state (~1 robot's distance
       * from the goal)
       */
      public static final double OUTTAKE_UPPER_FAST = -0.565;
      /**
       * the percent to run the lower outtake during the outtake fast state (~1 robot's distance
       * from the goal)
       */
      public static final double OUTTAKE_LOWER_FAST = -0.275;
    }

    public static final class Ports {
      public static final int LOWER_MOTOR = 15;
      public static final int UPPER_MOTOR = 8;
      public static final int RIGHT_EJECT_MOTOR = 16;
      public static final int LEFT_EJECT_MOTOR = 9;
    }
  }
}
