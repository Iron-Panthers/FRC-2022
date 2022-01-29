package frc.util;

import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.BooleanSupplier;

public class Util {
  private Util() {}

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return (a - epsilon <= b) && (a + epsilon >= b);
  }

  public static boolean epsilonZero(double a, double epsilon) {
    return epsilonEquals(a, 0, epsilon);
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

  public static double relativeAngularDifference(Rotation2d currentAngle, double newAngle) {
    return relativeAngularDifference(currentAngle.getDegrees(), newAngle);
  }

  /**
   * Compares a layer button with a layer state, and if they match returns the layered button
   *
   * @param layer the boolean supplier that is the layer switch
   * @param layerState the state of the layer switch that is valid
   * @param button the button inside the layer
   * @return true if the layer is enabled and the button is pressed
   */
  public static BooleanSupplier cumBooleanSupplier(
      BooleanSupplier layer, boolean layerState, BooleanSupplier button) {
    return () -> layer.getAsBoolean() == layerState && button.getAsBoolean();
  }
}
