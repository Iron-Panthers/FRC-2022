package frc.robot.autonomous;

import java.awt.*;
import java.awt.datatransfer.*;
import java.util.Scanner;

public class ReplaceFalconCode {
  private static void println(String v) {
    System.out.println(v);
  }

  private static String clipboard() {
    try {
      return (String)
          Toolkit.getDefaultToolkit().getSystemClipboard().getData(DataFlavor.stringFlavor);

    } catch (Exception e) {
      return "";
    }
  }

  public static void main(String[] args) {
    Scanner in = new Scanner(System.in);

    println("copy the falcon code, then press enter.");
    in.nextLine();

    println(clipboard());
  }
}
