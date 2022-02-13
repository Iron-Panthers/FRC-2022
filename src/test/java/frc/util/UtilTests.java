package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;

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
    assertEquals(0, Util.angleSnap(0, snaps2));
    assertEquals(45, Util.angleSnap(56.5, snaps2));
    assertEquals(270, Util.angleSnap(265, snaps1));
    assertEquals(270, Util.angleSnap(290, snaps1));
    assertEquals(350, Util.angleSnap(360, snaps1));
  }
}
