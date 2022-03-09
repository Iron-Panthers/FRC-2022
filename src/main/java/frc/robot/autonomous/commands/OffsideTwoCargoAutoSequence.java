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

    // Sequence is explained by comments
    addCommands(
        new InstantCommand(() -> drivebaseSubsystem.zeroGyroscope(), drivebaseSubsystem),
        // If arm not properly up, try to jog it. Should be addressed physically soon.
        deadline(
            new WaitCommand(0.75),
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_LOW_POSITION), armSubsystem)),
        // Score pre-load
        deadline(
            new WaitCommand(0.75 /* secs */),
            new IntakeCommand(IntakeSubsystem.Modes.OUTTAKE, intakeSubsystem)),
        // Follow trajectory and intake when we are near our target cargo
        parallel(
            sequence(
                new WaitCommand(0.5),
                new InstantCommand(
                    () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem)),
            deadline(
                followTrajectory,
                sequence(
                    new WaitCommand(0.5),
                    deadline(
                        new WaitCommand(2),
                        new IntakeCommand(IntakeSubsystem.Modes.INTAKE, intakeSubsystem))))),
        // Once we're back at the start pose, raise the arm to the scoring position
        deadline(
            new WaitCommand(2),
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_LOW_POSITION), armSubsystem)),
        // Score the 1 cargo
        deadline(
            new WaitCommand(1 /* sec */),
            new IntakeCommand(IntakeSubsystem.Modes.OUTTAKE, intakeSubsystem)));
  }
}
