package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilParamTest;
import frc.UtilTest;
import java.util.stream.Stream;
import org.junit.jupiter.params.provider.Arguments;
import org.junit.jupiter.params.provider.MethodSource;

public class UtilTests {
  @UtilTest
  public void epsilonReturnsTrueWhenEqual() {
    assertTrue(Util.epsilonEquals(2, 2.1, .2));
    assertTrue(Util.epsilonEquals(2, 2.1, .1));
  }

  @UtilTest
  public void epsilonReturnsFalseWhenUnequal() {
    assertFalse(Util.epsilonEquals(2, 2.1, 1e-2));
    assertFalse(Util.epsilonEquals(5, 2, 1));
  }

  @UtilTest
  public void relativeAngularDifferenceZerosOnEquivalent() {
    assertEquals(0, Util.relativeAngularDifference(100, 100), 1e-9);
    assertEquals(0, Util.relativeAngularDifference(0, 360 * 2), 1e-9);
    assertEquals(0, Util.relativeAngularDifference(0, 0), 1e-9);
  }

  @UtilTest
  public void relativeAngularDifferenceTakesShortestPath() {
    assertEquals(-100, Util.relativeAngularDifference(0, 100));
    assertEquals(-100, Util.relativeAngularDifference(360, 360 + 100));
    assertEquals(60, Util.relativeAngularDifference(360, 360 + 300));
    assertEquals(179, Util.relativeAngularDifference(0, 181));
    assertEquals(-170, Util.relativeAngularDifference(0, 170));
    assertEquals(-90, Util.relativeAngularDifference(270, 360));
    assertEquals(-90, Util.relativeAngularDifference(270, 0));
  }

  @UtilTest
  public void vectorToAngleCorrect() {
    assertEquals(0, Util.vectorToAngle(0, 0));
    assertEquals(45, Util.vectorToAngle(1, 1));
    assertEquals(270, Util.vectorToAngle(0, -5));
    assertEquals(359, Util.vectorToAngle(1, -.015), .2);
  }

  @UtilTest
  public void angleSnapCorrect() {
    double[] snaps1 = {90, 270, 350};
    double[] snaps2 = {0, 45, 90, 135, 180, 225, 270, 315};
    assertEquals(0, Util.angleSnap(0, snaps2), "exact match snaps");
    assertEquals(45, Util.angleSnap(56.5, snaps2));
    assertEquals(270, Util.angleSnap(265, snaps1));
    assertEquals(270, Util.angleSnap(290, snaps1));
    assertEquals(350, Util.angleSnap(360, snaps1));
    assertEquals(0, Util.angleSnap(5, snaps2), "close match snaps down");
    assertEquals(
        0, Util.angleSnap(355, snaps2), "snaps to closer angle, even if its on a different level");
  }

  public static Stream<Arguments> vectorMagnitudeProvider() {
    double sqrt2 = Math.sqrt(2);
    double sqrt3 = Math.sqrt(3);
    return Stream.of(
        Arguments.of(1, 0, 1),
        Arguments.of(sqrt2 / 2, sqrt2 / 2, 1),
        Arguments.of(sqrt3 / 2, .5, 1),
        Arguments.of(-sqrt2 / 2, sqrt2 / 2, 1),
        Arguments.of(-sqrt2 / 2, -sqrt2 / 2, 1),
        Arguments.of(sqrt2, sqrt2, 2),
        Arguments.of(10, 0, 10));
  }

  @UtilParamTest
  @MethodSource("vectorMagnitudeProvider")
  public void vectorMagnitudeCorrect(double x, double y, double magnitude) {
    assertEquals(
        magnitude,
        Util.vectorMagnitude(x, y),
        1e-9,
        String.format("||<%s, %s>|| == %s", x, y, magnitude));
  }
}
