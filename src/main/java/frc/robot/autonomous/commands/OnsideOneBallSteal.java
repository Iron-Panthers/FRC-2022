// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.autonomous.Waypoints.OffsideStartToCenterCargo;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceIntakeModeCommand;
import frc.robot.commands.RotateAngleDriveCommand;
import frc.robot.commands.SetIntakeModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;

public class OnsideOneBallSteal extends SequentialCommandGroup {
  /** Creates a new OffsideTwoCargoAutoSequence. */
  public OnsideOneBallSteal(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      SwerveDriveKinematics kinematics,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem) {
    // Create TrajectoryConfig object
    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
            .setKinematics(kinematics);

    trajectoryConfig.setReversed(false);

    Trajectory offsideStartToCenterCargo =
        TrajectoryGenerator.generateTrajectory(
            OffsideStartToCenterCargo.FIRST,
            List.of(),
            OffsideStartToCenterCargo.LAST,
            trajectoryConfig);

    trajectoryConfig.setReversed(true);

    Trajectory centerCargoToOffsideStart =
        TrajectoryGenerator.generateTrajectory(
            OffsideStartToCenterCargo.LAST,
            List.of(),
            OffsideStartToCenterCargo.FIRST,
            trajectoryConfig);

    // Sequence is explained by comments
    addCommands(
        // Score preload
        deadline(
            new WaitCommand(0.75 /* secs */),
            new SetIntakeModeCommand(
                intakeSubsystem, IntakeSubsystem.Modes.ALIGN_HIGH, IntakeSubsystem.Modes.OFF)),
        // Follow trajectory and intake when we are near our target cargo
        parallel(
            sequence(
                new WaitCommand(1),
                new InstantCommand(
                    () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem)),
            deadline(
                new FollowTrajectoryCommand(offsideStartToCenterCargo, true, drivebaseSubsystem),
                sequence(
                    new WaitCommand(0.25),
                    new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE)))),
        // keeping arm down, rotate to 0 degree
        deadline(
            new WaitCommand(3),
            new RotateAngleDriveCommand(drivebaseSubsystem, () -> 0, () -> 0, 0)));
  }
}