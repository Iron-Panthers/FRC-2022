package frc.util;

import edu.wpi.first.wpilibj.XboxController;

public class DPadCluster {
  XboxController controller;

  public DPadCluster(XboxController controller) {
    this.controller = controller;
  }

  private static enum Direction {
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    int angle;

    private Direction(int angle) {
      this.angle = angle;
    }
  }

  private boolean get(Direction direction) {
    int currentAngle = controller.getPOV();
    return Math.abs(Util.relativeAngularDifference(direction.angle, currentAngle)) <= 45;
  }

  public boolean getUpButton() {
    return get(Direction.UP);
  }

  public boolean getRightButton() {
    return get(Direction.RIGHT);
  }

  public boolean getDownButton() {
    return get(Direction.DOWN);
  }

  public boolean getLeftButton() {
    return get(Direction.LEFT);
  }
}
