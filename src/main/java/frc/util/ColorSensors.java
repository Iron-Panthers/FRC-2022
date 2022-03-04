package frc.util;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensors {
  private final I2C.Port port;

  private final ColorMatch colorMatcher = new ColorMatch();

  private ColorSensorV3 colorsensor3;

  private final Color kBlueTarget = new Color(0.143, 0.427, 0.429);

  private final Color kRedTarget = new Color(0.561, 0.232, 0.114);

  public ColorSensors(I2C.Port port, ColorSensorV3 colorsensor3) {
    this.port = port;
    this.colorsensor3 = colorsensor3;
    colorMatcher.addColorMatch(kBlueTarget);
    colorMatcher.addColorMatch(kRedTarget);
  }

  enum Colors {
    RED,
    BLUE,
    NONE;
  }

  public Colors getValue() {

    if (colorsensor3.getProximity() <= 240) {
      Color detectedColor = colorsensor3.getColor();

      ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

      /*
      If the ball matches the target color we set before, it will return the enum for the color.
      There are only two  options for color, if there is nothing there (the balls being out of proximity) it will return none.
      We are using the method of matchClosestColor to match the colors.
      */

      if (match.color == kRedTarget) {
        return Colors.RED;
      } else {
        return Colors.BLUE;
      }
    } else {
      return Colors.NONE;
    }
  }
}
