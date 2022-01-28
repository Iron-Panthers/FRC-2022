package frc.util;

public class Util {
  private Util() {}

  private static double kEpsilon = 1e-5;

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  /**
   * Scales radial deadband
   *
   * <p>The deadband value is set to be the new zero
   *
   * @param value the raw value
   * @param deadband the deadband range
   * @return the value with radial deadband applied
   */
  public static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
      return 0.0;
    }
  }

  /**
   * This function finds the degree difference between angles, the shortest path. useful for pid
   * control of drivebase rotation
   *
   * @param currentAngle Current Angle Degrees
   * @param newAngle Target Angle Degrees
   * @return Shortest angular difference in degrees
   */
  public static double relativeAngularDifference(double currentAngle, double newAngle) {
    currentAngle %= 360;
    newAngle %= 360;
    double negDifference = currentAngle - newAngle;
    return negDifference < -180 ? 360 + negDifference : negDifference;
  }
}
