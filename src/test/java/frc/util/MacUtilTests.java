package frc.util;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.UtilTest;
import java.util.List;

public class MacUtilTests {

  private byte[] macOne = {46, -75, -30, 99, 104, 4}; // 2E:B5:E2:63:68:04
  private byte[] macTwo = {94, 68, 23, 53, 15, -37}; // 5E:44:17:35:0F:DB

  private List<byte[]> addressBytes =
      List.of(macOne, macTwo); // test output of MacUtil.getMacAddresses();

  @UtilTest
  public void getMacAddressesDoesNotThrow() {
    assertDoesNotThrow(() -> MacUtil.getMacAddresses());
  }

  @UtilTest
  public void macAddressIsFormattedProperly() {
    assertEquals("2E:B5:E2:63:68:04", MacUtil.macToString(macOne));
    assertEquals("5E:44:17:35:0F:DB", MacUtil.macToString(macTwo));
  }
}
