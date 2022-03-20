// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.Constants.Drive.Dims;
import frc.robot.autonomous.Waypoints.OnsideStartToInnerCargoAndBack;
import java.util.List;

/**
 * This class exists only to test the total time it takes to track a given trajectory, based on the
 * robot's kinematics, a given max velocity constraint (in meters per second), and a given max
 * acceleration constraint (in meters per second squared). This class should not be called in the
 * robot program.
 */
public class BinTrajectoryParameterRuntimeTest {
  private BinTrajectoryParameterRuntimeTest() {
    throw new UnsupportedOperationException(
        "don't construct an instance of BinTrajectoryParameterRuntimeTest, it should never be referenced in the robot program");
  }

  public static void main(String[] a) {
    // Declare the robot's kinematics
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

    // Declare the trajectory movement constraints
    final double maxVelocityMetersPerSecond = 4;
    final double maxAccelerationMetersPerSecondSq = 2;

    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
            .setKinematics(kinematics);

    // You may want to consider adding additional configurations/constraints for testing purposes

    Trajectory testTrajectory =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToInnerCargoAndBack.FIRST,
            List.of(OnsideStartToInnerCargoAndBack.MIDDLE_B),
            OnsideStartToInnerCargoAndBack.LAST,
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
