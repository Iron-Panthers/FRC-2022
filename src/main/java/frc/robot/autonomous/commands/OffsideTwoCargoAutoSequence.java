// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autonomous.commands;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.autonomous.Waypoints.OffsideStartToCenterCargoAndBack;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;

public class OffsideTwoCargoAutoSequence extends SequentialCommandGroup {
  /** Creates a new OffsideTwoCargoAutoSequence. */
  public OffsideTwoCargoAutoSequence(
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

    Trajectory offsideStartToCenterCargoAndBack =
        TrajectoryGenerator.generateTrajectory(
            OffsideStartToCenterCargoAndBack.FIRST,
            List.of(OffsideStartToCenterCargoAndBack.MIDDLE_B),
            OffsideStartToCenterCargoAndBack.LAST,
            trajectoryConfig);

    // Create command to follow the trajectory
    // Note: this command will localize the odometry to the starting position (offside center
    // tarmac)
    Command followTrajectory =
        new FollowTrajectoryCommand(offsideStartToCenterCargoAndBack, true, drivebaseSubsystem);

    addCommands(
        new InstantCommand(() -> drivebaseSubsystem.zeroGyroscope(), drivebaseSubsystem),
        deadline(
            new WaitCommand(0.5),
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_LOW_POSITION), armSubsystem)),
        deadline(
            new WaitCommand(1.0 /* secs */),
            new StartEndCommand(
                () -> intakeSubsystem.setMode(IntakeSubsystem.Modes.OUTTAKE),
                intakeSubsystem::nextMode,
                intakeSubsystem)),
        new InstantCommand(
            () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem),
        deadline(
            followTrajectory, new IntakeCommand(IntakeSubsystem.Modes.INTAKE, intakeSubsystem)),
        deadline(
            new WaitCommand(1.5),
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_LOW_POSITION), armSubsystem)),
        race(
            new IntakeCommand(IntakeSubsystem.Modes.OUTTAKE, intakeSubsystem),
            new WaitCommand(2.0 /* secs */)));
  }
}
