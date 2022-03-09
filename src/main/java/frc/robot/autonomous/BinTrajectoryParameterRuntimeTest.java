package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.Drive.Dims;
import frc.robot.autonomous.Waypoints.OffsideStartToCenterCargoAndBack;
import java.util.List;

public class BinTrajectoryParameterRuntimeTest {
  public static void main(String[] a) {
    SwerveDriveKinematics kinematics =
        new SwerveDriveKinematics(
            // Front right
            new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0),
            // Front left
            new Translation2d(Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
            // Back left
            new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, Dims.WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-Dims.TRACKWIDTH_METERS / 2.0, -Dims.WHEELBASE_METERS / 2.0));

    final double maxVelocityMetersPerSecond = 3;
    final double maxAccelerationMetersPerSecondSq = 1.5;

    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
            .setKinematics(kinematics);

    Trajectory testTrajectory =
        TrajectoryGenerator.generateTrajectory(
            OffsideStartToCenterCargoAndBack.FIRST,
            List.of(OffsideStartToCenterCargoAndBack.MIDDLE_B),
            OffsideStartToCenterCargoAndBack.LAST,
            trajectoryConfig);

    System.out.println(
        "running with trajectory configuration:\nmax_velocity (m/s): "
            + maxVelocityMetersPerSecond
            + "\nmax_acceleration (m/s2): "
            + maxAccelerationMetersPerSecondSq);

    var trajectoryTime = testTrajectory.getTotalTimeSeconds();

    System.out.println("test_trajectory time (seconds): " + trajectoryTime);

    System.out.println(
        "remaining time in autonomous after following test_trajectory: " + (15d - trajectoryTime));
  }
}
