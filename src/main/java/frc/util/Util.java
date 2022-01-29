package frc.util;

import edu.wpi.first.math.geometry.Rotation2d;

public class Util {
  private Util() {}

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonZero(double a, double epsilon) {
    return epsilonEquals(a, 0, epsilon);
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

  public static double relativeAngularDifference(Rotation2d currentAngle, double newAngle) {
    return relativeAngularDifference(currentAngle.getDegrees(), newAngle);
  }
}
