package frc.util;

import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

public class MacUtil {
  private static void logErr(SocketException e) {
    System.out.print(
        "mac util, which is used to toggle const values based on the robot, threw SocketException: ");
    System.out.println(e);
  }

  public static List<byte[]> getMacAddresses() {
    List<byte[]> macAddresses = new ArrayList<>();

    // init dance to not throw an error
    Enumeration<NetworkInterface> networkInterfaces;

    try {
      networkInterfaces = NetworkInterface.getNetworkInterfaces();
    } catch (SocketException e) {
      logErr(e);
      return macAddresses;
    }

    while (networkInterfaces.hasMoreElements()) {
      try { // get hardware address can throw
        byte[] address = networkInterfaces.nextElement().getHardwareAddress();

        // the address may be null
        if (address == null) continue;

        macAddresses.add(address);

      } catch (SocketException e) {
        logErr(e);
      }
    }

    return macAddresses;
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
