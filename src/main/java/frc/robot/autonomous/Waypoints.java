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
    public static final Pose2d FIRST = new Pose2d(7.010, 4.572, Rotation2d.fromDegrees(180 + 180));
    public static final Translation2d MIDDLE_B = new Translation2d(6, 4.572);
    public static final Pose2d LAST = new Pose2d(4.926, 4.572, Rotation2d.fromDegrees(170));
  }

  public static final class OffsideStartToCenterCargo {

    public static final Pose2d FIRST = new Pose2d(7.010, 4.572, Rotation2d.fromDegrees(160.0));
    public static final Pose2d LAST = new Pose2d(4.9987, 6.096, Rotation2d.fromDegrees(150));
  }

  public static final class OnsideStartToOuterCargoAndBack {
    public static final Pose2d FIRST =
        new Pose2d(
            7.772 /* meters (25.5 feet) */,
            2.896 /* meters (9.5 feet) */,
            Rotation2d.fromDegrees(251 + 180.0));
    public static final Translation2d MIDDLE_B =
        new Translation2d(7.524 /* meters (24.685 feet) */, 0.653 /* meters (2.142 feet) */);
    public static final Pose2d LAST =
        new Pose2d(
            7.772 /* meters (25.5 feet) */,
            2.896 /* meters (9.5 feet) */,
            Rotation2d.fromDegrees(69));
  }

  public static final class OnsideStartToInnerCargoAndBack {
    public static final Pose2d FIRST =
        new Pose2d(
            7.772 /* meters (25.5 feet) */,
            2.896 /* meters (9.5 feet) */,
            Rotation2d.fromDegrees(251 + 180.0));
    public static final Translation2d MIDDLE_B =
        new Translation2d(5.182 /* meters (17 feet) */, 1.890 /* meters (6.2 feet) */);
    public static final Pose2d LAST =
        new Pose2d(
            7.772 /* meters (25.5 feet) */,
            2.896 /* meters (9.5 feet) */,
            Rotation2d.fromDegrees(69));
  }

  public static final class OnsideStartToTerminalTwoCargoAndBack {
    public static final Pose2d FIRST =
        new Pose2d(
            7.772 /* meters (25.5 feet) */,
            2.896 /* meters (9.5 feet) */,
            Rotation2d.fromDegrees(251));
    public static final Translation2d MIDDLE_B =
        new Translation2d(1.320 /* meters (4.332 feet) */, 1.184 /* meters (3.886 feet) */);
    public static final Pose2d LAST =
        new Pose2d(
            7.772 /* meters (25.5 feet) */,
            2.896 /* meters (9.5 feet) */,
            Rotation2d.fromDegrees(69));
  }
}
