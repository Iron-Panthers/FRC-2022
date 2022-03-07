package frc.robot.autonomous;

import java.awt.*;
import java.awt.datatransfer.*;
import java.util.Scanner;
import java.util.Stack;
import java.util.function.Function;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

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

  public static void main(String[] args) {
    Scanner in = new Scanner(System.in);

    println("copy the falcon code, then press enter.");
    in.nextLine();

    String code = clipboard();
    code = code.replace("wayPoints = listOf(\n", "");
    code = code.replace("\n),", "");
    code = code.replace("    ", "");

    code =
        replaceTextOfMatchGroup(
            code,
            Pattern.compile("(\\d*\\.\\d*)\\.feet", Pattern.MULTILINE),
            1,
            (feet) -> {
              return String.format("%s /*meters, converted from %s feet*/", feet + 2, feet);
            });
    code = code.replace(".feet", "");

    println(code);
  }
}
