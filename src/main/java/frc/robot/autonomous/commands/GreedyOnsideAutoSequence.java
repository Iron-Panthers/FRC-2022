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
import frc.robot.autonomous.Waypoints.OnsideStartToInnerCargo;
import frc.robot.autonomous.Waypoints.OnsideStartToOuterCargo;
import frc.robot.autonomous.Waypoints.OnsideStartToTerminalTwoCargo;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceIntakeModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;

public class GreedyOnsideAutoSequence extends SequentialCommandGroup {
  /** Creates a new GreedyOnsideAutoSequence. */
  public GreedyOnsideAutoSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      SwerveDriveKinematics kinematics,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem) {
    // Create TrajectoryConfig object
    TrajectoryConfig trajectoryConfig =
        new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
            .setKinematics(kinematics)
            .setReversed(false);

    Trajectory onsideToOuterCargo =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToOuterCargo.FIRST,
            List.of(),
            OnsideStartToOuterCargo.LAST,
            trajectoryConfig);

    Trajectory onsideToInnerCargo =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToInnerCargo.FIRST,
            List.of(),
            OnsideStartToInnerCargo.LAST,
            trajectoryConfig);

    Trajectory onsideToTerminalTwoCargo =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToTerminalTwoCargo.FIRST,
            List.of(),
            OnsideStartToTerminalTwoCargo.LAST,
            trajectoryConfig);

    //
    // THE PATHS BACK
    //
    trajectoryConfig.setReversed(true);

    Trajectory outerCargoToOnside =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToOuterCargo.LAST,
            List.of(),
            OnsideStartToOuterCargo.FIRST,
            trajectoryConfig);

    Trajectory innerCargoToOnside =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToInnerCargo.LAST,
            List.of(),
            OnsideStartToInnerCargo.FIRST,
            trajectoryConfig);

    Trajectory terminalTwoCargoToOnside =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToTerminalTwoCargo.LAST,
            List.of(),
            OnsideStartToTerminalTwoCargo.FIRST,
            trajectoryConfig);

    // Sequence is explained by comments
    addCommands(
        new InstantCommand(() -> drivebaseSubsystem.zeroGyroscope(), drivebaseSubsystem),
        // Follow trajectory and intake after a delay
        // Lower arm once so as to not induce too much dive at the beginning
        parallel(
            sequence(
                new WaitCommand(0.8), // TODO: tune
                new InstantCommand(
                    () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem)),
            deadline(
                new FollowTrajectoryCommand(onsideToOuterCargo, true, drivebaseSubsystem),
                sequence(
                    new WaitCommand(0.25),
                    new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE)))),
        new FollowTrajectoryCommand(outerCargoToOnside, drivebaseSubsystem),
        // Once we're back at the start pose, raise the arm to the scoring position
        deadline(
            new WaitCommand(0.75),
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_HIGH_POSITION - 10),
                armSubsystem)),
        // Score the 2 cargo
        deadline(
            new WaitCommand(0.5 /* sec */),
            new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.OUTTAKE)),
        // GOTO INNER CARGO
        parallel(
            sequence(
                new WaitCommand(0.8), // TODO: tune
                new InstantCommand(
                    () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem)),
            deadline(
                new FollowTrajectoryCommand(onsideToInnerCargo, true, drivebaseSubsystem),
                sequence(
                    new WaitCommand(0.25),
                    new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE)))),
        new FollowTrajectoryCommand(innerCargoToOnside, drivebaseSubsystem),
        // SCORE INNER CARGO
        deadline(
            new WaitCommand(0.5 /* sec */),
            new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.OUTTAKE)),
        // GOTO TERMINAL
        parallel(
            sequence(
                new WaitCommand(1), // TODO: tune
                new InstantCommand(
                    () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem)),
            deadline(
                new FollowTrajectoryCommand(onsideToTerminalTwoCargo, true, drivebaseSubsystem),
                sequence(
                    new WaitCommand(0.25),
                    new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.INTAKE)))),
        new FollowTrajectoryCommand(terminalTwoCargoToOnside, drivebaseSubsystem),
        // no way this happens in time with this revision, but...
        // SCORE TERMINAL CARGO
        deadline(
            new WaitCommand(0.5 /* sec */),
            new ForceIntakeModeCommand(intakeSubsystem, IntakeSubsystem.Modes.OUTTAKE)));
  }
}
