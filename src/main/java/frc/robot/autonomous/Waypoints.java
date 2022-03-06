package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Waypoints {
  private Waypoints() {
    throw new UnsupportedOperationException(
        "don't construct an instance of Waypoints, a static helper class");
  }

  // Baseline autonomous
  public static final Pose2d BASELINE_START_METERS =
      new Pose2d(5.9, 3.855, Rotation2d.fromDegrees(0));
  public static final Translation2d BASELINE_LINEARIZE_POINT_ONE = new Translation2d(5, 3.855);
  public static final Pose2d BASELINE_END_METERS =
      new Pose2d(3.61, 3.855, Rotation2d.fromDegrees(10));
}
