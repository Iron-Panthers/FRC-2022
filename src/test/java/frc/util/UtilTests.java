package frc.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.UtilTest;

public class UtilTests {
  @UtilTest
  public void deadbandScalesCorrectly() {
    assertEquals(1.0, Util.deadband(1.0, .3), "deadband should still max out");
    assertEquals(0.0, Util.deadband(.1, .2), "value should be zero when inside deadband");
    assertEquals(
        0.2,
        Util.deadband(.6, .5),
        1e-5 // epsilon for equality assertion
        ,
        "value should scale such that range from deadband to 1 is mapped to 0 to 1");
  }

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
    assertEquals(0, Util.relativeAngularDifference(100, 100));
    assertEquals(0, Util.relativeAngularDifference(0, 360 * 2));
  }

  @UtilTest
  public void relativeAngularDifferenceTakesShortestPath() {
    assertEquals(-100, Util.relativeAngularDifference(360, 360 + 100));
    assertEquals(60, Util.relativeAngularDifference(360, 360 + 300));
    assertEquals(179, Util.relativeAngularDifference(0, 181));
    assertEquals(-170, Util.relativeAngularDifference(0, 170));
  }
}
