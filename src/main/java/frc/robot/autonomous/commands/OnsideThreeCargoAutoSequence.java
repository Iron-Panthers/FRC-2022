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
import frc.robot.autonomous.Waypoints.OnsideStartToInnerCargoAndBack;
import frc.robot.autonomous.Waypoints.OnsideStartToOuterCargoAndBack;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.List;

// SAME AS TWO CARGO VERSION EXCEPT IT GRABS THE ONSIDE INNER AND SCORES 1 AFTER
//
// Feel free to ignore the contents below, these are just some notes I want to check in:
// Autonomous runtime of 15s should theoretically hold the values provided v_max=3 and a_max=1.5
// By Trajectory runtime binary program in the package root:
// Outer path runtime = 5.81 s approx
// Inner path runtime = 5.18 s approx
// This amounts to around 11 s of path runtime, so remaining time devoted to other actions cannot
// add up to >=4s
//
// 0.5 + 0.75 + 0.5 + 0.75 + 0.5 = 3 s
// we are 1 second short (oops :P)
// 4/5 cargo auto sequence require faster speed?
// (3, 1.5) -> ~14s total
// (4, 2) -> ~ 4.96 + 4.37 --> we shave around 1.5s total (it's not much better)
// so, probably requires path reform more so than speed (unless we push acceleration limits)
// probably just run the 3,1.5 configuration is best unless we fundamentally change what we track
public class OnsideThreeCargoAutoSequence extends SequentialCommandGroup {
  /** Creates a new OnsideThreeCargoAutoSequence. */
  public OnsideThreeCargoAutoSequence(
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

    Trajectory onsideStartToInnerCargoAndBack =
        TrajectoryGenerator.generateTrajectory(
            OnsideStartToInnerCargoAndBack.FIRST,
            List.of(OnsideStartToInnerCargoAndBack.MIDDLE_B),
            OnsideStartToInnerCargoAndBack.LAST,
            trajectoryConfig);

    // create the second follow command, which doesn't localize
    // this is run after scoring the first two cargo
    Command followOnsideToInnerTrajectory =
        new FollowTrajectoryCommand(onsideStartToInnerCargoAndBack, drivebaseSubsystem);

    // Sequence is explained by comments
    addCommands(
        new InstantCommand(() -> drivebaseSubsystem.zeroGyroscope(), drivebaseSubsystem),
        // Lower arm to bottom (TODO: look into a delayed lower)
        new InstantCommand(
            () -> armSubsystem.setAngle(Arm.Setpoints.INTAKE_POSITION), armSubsystem),
        // Follow the first trajectory which takes us back to start
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
            new IntakeCommand(IntakeSubsystem.Modes.OUTTAKE, intakeSubsystem)),
        // Follow the second trajectory which takes us back to start
        deadline(
            followOnsideToInnerTrajectory,
            new IntakeCommand(IntakeSubsystem.Modes.INTAKE, intakeSubsystem)),
        // Once we're back at the start pose, raise the arm to the scoring position
        deadline(
            new WaitCommand(0.75),
            new InstantCommand(
                () -> armSubsystem.setAngle(Arm.Setpoints.OUTTAKE_LOW_POSITION), armSubsystem)),
        // Score the 1 cargo
        deadline(
            new WaitCommand(0.5 /* sec */),
            new IntakeCommand(IntakeSubsystem.Modes.OUTTAKE, intakeSubsystem)));
  }
}
