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
    double difference1 = Math.abs(currentAngle - newAngle);
    double difference2 = Math.abs(360 - difference1);
    double difference = difference1 < difference2 ? difference1 : difference2;

    if ((currentAngle + difference) % 360 == newAngle) return difference;
    return difference * -1;
  }

  public static double relativeAngularDifference(Rotation2d currentAngle, double newAngle) {
    return relativeAngularDifference(currentAngle.getDegrees(), newAngle);
  }
}
