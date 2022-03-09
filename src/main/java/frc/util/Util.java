// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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

    if ((currentAngle + difference) % 360 == newAngle) return difference * -1;
    return difference;
  }

  public static double relativeAngularDifference(Rotation2d currentAngle, double newAngle) {
    return relativeAngularDifference(currentAngle.getDegrees(), newAngle);
  }

  /**
   * turn x and y of a vector to a [0, 360] angle
   *
   * @param x x value of vector
   * @param y y value of vector
   * @return [0, 360] mapped angle of vector
   */
  public static double vectorToAngle(double x, double y) {
    double angle = Math.atan2(y, x);
    return (angle * (180 / Math.PI) + 360) % 360;
  }

  /**
   * snap an angle to the closest angle in an array of angles
   *
   * @param angle angle to be snapped
   * @param snaps array of angles to snap to
   * @return closest angle in snap array
   */
  public static double angleSnap(double angle, double[] snaps) {
    double closest = snaps[0];
    for (double snap : snaps) {
      if (Math.abs(relativeAngularDifference(angle, snap))
          < Math.abs(relativeAngularDifference(angle, closest))) {
        closest = snap;
      }
    }
    return closest;
  }

  public static double vectorMagnitude(double x, double y) {
    return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
  }
}
