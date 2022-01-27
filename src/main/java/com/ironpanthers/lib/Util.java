package com.ironpanthers.lib;

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
}
