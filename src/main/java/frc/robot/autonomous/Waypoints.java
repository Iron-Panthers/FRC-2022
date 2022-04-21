package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

@SuppressWarnings("java:S1118")
public final class Waypoints {
  private Waypoints() {
    throw new UnsupportedOperationException(
        "don't construct an instance of Waypoints, a static helper class");
  }

  public static final class Baseline {
    public static final Pose2d FIRST = new Pose2d(7.010, 4.572, Rotation2d.fromDegrees(180));
    public static final Translation2d MIDDLE_B = new Translation2d(6, 4.572);
    public static final Pose2d LAST = new Pose2d(4.926, 4.572, Rotation2d.fromDegrees(170));
  }

  private static final Pose2d DEFAULT_OFFSIDE_START =
      new Pose2d(7.010, 4.572, Rotation2d.fromDegrees(160.0));

  private static final Pose2d DEFAULT_ONSIDE_START =
      new Pose2d(7.772, 2.896, Rotation2d.fromDegrees(-110));

  public static final class OffsideStartToCenterCargo {
    public static final Pose2d FIRST = DEFAULT_OFFSIDE_START;
    // OFFSIDE CARGO RING SPOT
    public static final Pose2d LAST = new Pose2d(4.9987, 6.096, Rotation2d.fromDegrees(150));
  }

  public static final class OnsideStartToOuterCargo {
    public static final Pose2d FIRST = DEFAULT_ONSIDE_START;
    // OUTER CARGO RING SPOT
    public static final Pose2d LAST = new Pose2d(7.55904, 0.48768, Rotation2d.fromDegrees(-90));
  }

  public static final class OnsideStartToInnerCargo {
    public static final Pose2d FIRST = DEFAULT_ONSIDE_START;
    // INNER CARGO RING SPOT
    public static final Pose2d LAST = new Pose2d(5.1816, 1.9812, Rotation2d.fromDegrees(-160));
  }

  public static final class OnsideStartToTerminalTwoCargo {
    public static final Pose2d FIRST = DEFAULT_ONSIDE_START;

    public static final Pose2d LAST = new Pose2d(1.18872, 1.295, Rotation2d.fromDegrees(-135));
  }
}
