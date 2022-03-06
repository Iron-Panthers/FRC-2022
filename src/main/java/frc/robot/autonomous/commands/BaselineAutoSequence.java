// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.autonomous.Waypoints.Baseline;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import java.util.List;

public class BaselineAutoSequence extends SequentialCommandGroup {
  public BaselineAutoSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      SwerveDriveKinematics kinematics,
      DrivebaseSubsystem drivebaseSubsystem) {
    // Create TrajectoryConfig object
    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
            .setKinematics(kinematics);

    // Generate the baseline trajectory
    Trajectory baselineTrajectory =
        TrajectoryGenerator.generateTrajectory(
            Baseline.START_METERS,
            List.of(Baseline.LINEARIZE_POINT_ONE),
            Baseline.END_METERS,
            trajectoryConfig);

    // Create command to follow the baseline trajectory
    Command followBaselineTrajectory =
        new FollowTrajectoryCommand(baselineTrajectory, true, drivebaseSubsystem);

    addCommands(followBaselineTrajectory);
  }
}
