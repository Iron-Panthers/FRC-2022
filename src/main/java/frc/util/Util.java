package frc.util;

public class Util {
  private Util() {}

  private static double kEpsilon = 1e-5;

  public static boolean epsilonEquals(double a, double b, double epsilon) {
    return Math.abs(a - b) <= epsilon;
  }

  public static boolean epsilonEquals(int a, int b, int epsilon) {
    return Math.abs(a - b) <= epsilon;
  }

  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, kEpsilon);
  }

  public static boolean epsilonEquals(int a, int b) {
    return epsilonEquals(a, b, kEpsilon);
  }
}