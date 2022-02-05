package frc.util;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;

public class MacUtilTests {

  private byte[] macOne = {46, -75, -30, 99, 104, 4}; // 2E:B5:E2:63:68:04
  private byte[] macTwo = {94, 68, 23, 53, 15, -37}; // 5E:44:17:35:0F:DB

  @UtilTest
  public void getMacAddressesDoesNotThrow() {
    assertDoesNotThrow(() -> MacUtil.getMacAddress());
  }

  @UtilTest
  public void macAddressIsFormattedProperly() {
    assertEquals("2E:B5:E2:63:68:04", MacUtil.macToString(macOne));
    assertEquals("5E:44:17:35:0F:DB", MacUtil.macToString(macTwo));
  }

  @UtilTest
  public void macAddressFormatsCorrectlyWithEmptyByteArray() {
    assertEquals("", MacUtil.macToString(new byte[0]));
    assertEquals("00:00:00:00:00:00", MacUtil.macToString(new byte[6]));
  }
}
