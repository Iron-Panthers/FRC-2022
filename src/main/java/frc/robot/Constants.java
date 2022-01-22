// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
            / Math.hypot(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0);

    public static final class Dims {
      // FIXME validate with hardware
      public static final double TRACKWIDTH_METERS =
          .5207; // 20.5 inches (source: cad) converted to meters
      public static final double WHEELBASE_METERS = TRACKWIDTH_METERS; // robot is square
    }

    public static final class Modules {
      public static final class FrontLeft {
        // FIXME add can id values
        public static final int DRIVE_MOTOR = 0;
        public static final int STEER_MOTOR = 0;
        public static final int STEER_ENCODER = 0;

        @SuppressWarnings("java:S2185") // this warning calls out unneeded math toRadians calls
        public static final double STEER_OFFSET =
            -Math.toRadians(0.0); // FIXME measure and set with hw
      }

      public static final class FrontRight {
        // FIXME add can id values
        public static final int DRIVE_MOTOR = 0;
        public static final int STEER_MOTOR = 0;
        public static final int STEER_ENCODER = 0;

        @SuppressWarnings("java:S2185") // this warning calls out unneeded math toRadians calls
        public static final double STEER_OFFSET =
            -Math.toRadians(0.0); // FIXME measure and set with hw
      }

      public static final class BackLeft {
        // FIXME add can id values
        public static final int DRIVE_MOTOR = 0;
        public static final int STEER_MOTOR = 0;
        public static final int STEER_ENCODER = 0;

        @SuppressWarnings("java:S2185") // this warning calls out unneeded math toRadians calls
        public static final double STEER_OFFSET =
            -Math.toRadians(0.0); // FIXME measure and set with hw
      }

      public static final class BackRight {
        // FIXME add can id values
        public static final int DRIVE_MOTOR = 0;
        public static final int STEER_MOTOR = 0;
        public static final int STEER_ENCODER = 0;

        @SuppressWarnings("java:S2185") // this warning calls out unneeded math toRadians calls
        public static final double STEER_OFFSET =
            -Math.toRadians(0.0); // FIXME measure and set with hw
      }
    }
  }
}
