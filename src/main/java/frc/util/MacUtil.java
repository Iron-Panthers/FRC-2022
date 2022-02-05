package frc.util;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

public class MacUtil {
  private MacUtil() {}

  /**
   * is the robot code is running on the competition robot? defaults to true, better to assume comp
   * then practice
   */
  public static final boolean IS_COMP_BOT =
      !macToString(getMacAddress())
          .equals(
              // this value is the mac address of the practice bot
              // if the read mac address is not the practice bot, we default to comp bot
              "2E:B5:E2:63:68:04" // FIXME: find actual mac address
              );

  private static void logErr(SocketException e) {
    System.out.print(
        "mac util, which is used to toggle const values based on the robot, threw SocketException: ");
    System.out.println(e);
  }

  public static byte[] getMacAddress() {

    // init dance to not throw an error
    Enumeration<NetworkInterface> networkInterfaces;

    try {
      networkInterfaces = NetworkInterface.getNetworkInterfaces();
    } catch (SocketException e) {
      logErr(e);
      return new byte[0];
    }

    while (networkInterfaces.hasMoreElements()) {
      try { // get hardware address can throw
        byte[] address = networkInterfaces.nextElement().getHardwareAddress();

        // the address may be null
        if (address == null) continue;

        return address;

      } catch (SocketException e) {
        logErr(e);
      }
    }

    // we couldn't read any network interfaces
    return new byte[0];
  }

  public static String macToString(byte[] address) {
    StringBuilder builder = new StringBuilder();
    for (int i = 0; i < address.length; i++) {
      if (i != 0) {
        builder.append(':');
      }
      builder.append(String.format("%02X", address[i]));
    }
    return builder.toString();
  }
}
