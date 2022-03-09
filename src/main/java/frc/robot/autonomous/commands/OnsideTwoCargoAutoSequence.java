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
import frc.robot.autonomous.Waypoints.OnsideStartToOuterCargoAndBack;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;

public class OnsideTwoCargoAutoSequence extends SequentialCommandGroup {
  /** Creates a new OnsideTwoCargoAutoSequence. */
  public OnsideTwoCargoAutoSequence(
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

    Trajectory onsideStartToOuterCargoAndBack =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToOuterCargoAndBack.FIRST,
            List.of(OnsideStartToOuterCargoAndBack.MIDDLE_B),
            OnsideStartToOuterCargoAndBack.LAST,
            trajectoryConfig);

    // create the follow command, which localizes (cuz it should)
    Command followOnsideToOuterTrajectory =
        new FollowTrajectoryCommand(onsideStartToOuterCargoAndBack, true, drivebaseSubsystem);

    // Sequence is explained by comments
    addCommands(
        new InstantCommand(() -> drivebaseSubsystem.zeroGyroscope(), drivebaseSubsystem),
        // Lower arm to bottom (TODO: look into a delayed lower)
        new InstantCommand(
            () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem),
        deadline(
            followOnsideToOuterTrajectory,
            sequence(
                new WaitCommand(0.5),
                deadline(
                    new WaitCommand(2),
                    new IntakeCommand(IntakeSubsystem.Modes.INTAKE, intakeSubsystem)))),
        // Once we're back at the start pose, raise the arm to the scoring position
        deadline(
            new WaitCommand(0.75),
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_LOW_POSITION), armSubsystem)),
        // Score the 2 cargo
        deadline(
            new WaitCommand(0.5 /* sec */),
            new IntakeCommand(IntakeSubsystem.Modes.OUTTAKE, intakeSubsystem)));
  }
}
