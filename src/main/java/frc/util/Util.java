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
   * This function makes a target angle relative to a new angle, for use in swerve code and other
   * situations where relative angles are easier to work with
   *
   * <p>lifted from:
   * https://github.com/frc1678/2910-clone/blob/b3c9e66cf49067b651ac658bd0c4406443e0ea86/src/main/java/com/team1678/lib/util/CTREModuleState.java#L32-L56
   *
   * @param scopeReference Current Angle, to make things relative
   * @param newAngle Target Angle
   * @return Closest angle within scope
   */
  private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
    double lowerBound;
    double upperBound;
    double lowerOffset = scopeReference % 360;
    if (lowerOffset >= 0) {
      lowerBound = scopeReference - lowerOffset;
      upperBound = scopeReference + (360 - lowerOffset);
    } else {
      upperBound = scopeReference - lowerOffset;
      lowerBound = scopeReference - (360 + lowerOffset);
    }
    while (newAngle < lowerBound) {
      newAngle += 360;
    }
    while (newAngle > upperBound) {
      newAngle -= 360;
    }
    if (newAngle - scopeReference > 180) {
      newAngle -= 360;
    } else if (newAngle - scopeReference < -180) {
      newAngle += 360;
    }
    return newAngle;
  }
}
