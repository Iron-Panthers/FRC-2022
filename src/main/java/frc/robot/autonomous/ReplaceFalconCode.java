package frc.robot.autonomous;

import java.awt.*;
import java.awt.datatransfer.*;
import java.util.Scanner;
import java.util.Stack;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

public class ReplaceFalconCode {
  private static final double FEET_IN_METER = 3.2808;
  private static final int DIGITS_PRECISION = 3;
  private static final int RESCAN_MS = 500;

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

  public static String replaceTextOfMatchGroup(
      String sourceString,
      Pattern pattern,
      int groupToReplace,
      Function<String, String> replaceStrategy) {
    Stack<Integer> startPositions = new Stack<>();
    Stack<Integer> endPositions = new Stack<>();
    Matcher matcher = pattern.matcher(sourceString);

    while (matcher.find()) {
      startPositions.push(matcher.start(groupToReplace));
      endPositions.push(matcher.end(groupToReplace));
    }
    StringBuilder sb = new StringBuilder(sourceString);
    while (!startPositions.isEmpty()) {
      int start = startPositions.pop();
      int end = endPositions.pop();
      if (start >= 0 && end >= 0) {
        sb.replace(start, end, replaceStrategy.apply(sourceString.substring(start, end)));
      }
    }
    return sb.toString();
  }

  public static String round(double num) {
    return String.format("%." + DIGITS_PRECISION + "f", num);
  }

  public static void main(String[] args) {
    Scanner in = new Scanner(System.in);

    println("copy the falcon code, then press enter.");
    in.nextLine();

    String code = clipboard();

    code = code.replace("wayPoints = listOf(\n", "");
    code = code.replace("\n),", "");
    code = code.replace("),", ")");
    code = code.replace("    ", "");
    code = code.replace("Pose2d", "new Pose2d");

    code =
        replaceTextOfMatchGroup(
            code,
            Pattern.compile("(\\d*\\.\\d*)\\.feet", Pattern.MULTILINE),
            1,
            (feet) -> {
              return String.format(
                  "%s /* meters (%s feet) */",
                  round(Double.parseDouble(feet) / FEET_IN_METER), feet);
            });
    code = code.replace(".feet", "");

    code =
        replaceTextOfMatchGroup(
            code,
            Pattern.compile("(-?\\d*?\\.?\\d*)\\.degrees", Pattern.MULTILINE),
            1,
            degrees -> {
              return String.format("Rotation2d.fromDegrees(%s)", degrees);
            });
    code = code.replace(".degrees", "");

    println(code);

    String[] points = code.split("\n");

    for (String point : points) {
      println("lol '" + point + "'\n");
    }
  }
}
