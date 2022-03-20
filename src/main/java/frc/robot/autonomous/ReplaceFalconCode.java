package frc.robot.autonomous;

import java.awt.*;
import java.awt.datatransfer.*;
import java.util.Scanner;
import java.util.Stack;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * This class converts the code output from FRC5190's FalconDashboard into a list of
 * public-static-final Pose2d and Translation2d waypoints, which are the critical points for path
 * generation.
 *
 * <p>The dashboard project can be found at: https://github.com/FRC5190/FalconDashboard
 *
 * <p>For more information on how this data is used to generate trajectories, see
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/trajectories/trajectory-generation.html#generating-the-trajectory
 */
public class ReplaceFalconCode {
  private static final double FEET_IN_METER = 3.2808;
  private static final int DIGITS_PRECISION = 3;
  private static final int RESCAN_MS = 500;

  private static final String[] ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZ".split("");

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

  private static void clipboard(String write) {
    try {
      var data = new StringSelection(write);

      Toolkit.getDefaultToolkit().getSystemClipboard().setContents(data, data);

    } catch (Exception e) {
      // we do not care
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
            Pattern.compile("(\\d*?\\.?\\d*)\\.feet", Pattern.MULTILINE),
            1,
            feet ->
                String.format(
                    "%s /* meters (%s feet) */",
                    round(Double.parseDouble(feet) / FEET_IN_METER), feet));
    code = code.replace(".feet", "");

    code =
        replaceTextOfMatchGroup(
            code,
            Pattern.compile("(-?\\d*?\\.?\\d*)\\.degrees", Pattern.MULTILINE),
            1,
            degrees -> String.format("Rotation2d.fromDegrees(%s)", degrees));
    code = code.replace(".degrees", "");

    String[] points = code.split("\n");

    StringBuilder output = new StringBuilder();

    for (int i = 0; i < points.length; i++) {

      output.append("public static final ");

      if (i == 0 || i == points.length - 1) {
        output.append("Pose2d ");
        output.append(i == 0 ? "FIRST " : "LAST ");
        output.append("= ");
        output.append(points[i]);
      } else {
        output.append("Translation2d ");
        output.append(String.format("MIDDLE_%S ", ALPHABET[i]));
        output.append("= ");
        String objectInit = points[i];
        objectInit = objectInit.replace("Pose2d", "Translation2d");
        objectInit = objectInit.replaceAll(Pattern.compile(", Rotation2d.*?\\)").toString(), "");
        output.append(objectInit);
      }
      output.append(";\n");
    }

    clipboard(output.toString());

    println(output.toString());
  }
}
