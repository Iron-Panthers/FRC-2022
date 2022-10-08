package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceIntakeModeCommand;
import frc.robot.commands.SetIntakeModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import java.util.function.DoubleFunction;

public class OnsideThreeBallSequence extends SequentialCommandGroup {
  public OnsideThreeBallSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      SwerveDriveKinematics kinematics,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem) {

    PathPlannerTrajectory threeBallOnsidePickup =
        PathPlanner.loadPath(
            "3ball onside pickup",
            maxAccelerationMetersPerSecondSq,
            maxAccelerationMetersPerSecondSq);

    PathPlannerTrajectory threeBallOnsideShoot =
        PathPlanner.loadPath(
            "3ball onside shoot",
            maxAccelerationMetersPerSecondSq,
            maxAccelerationMetersPerSecondSq);

    DoubleFunction<InstantCommand> armAngleCommand =
        angle -> new InstantCommand(() -> armSubsystem.setAngle(angle), armSubsystem);

    addCommands(
        new SetIntakeModeCommand(intakeSubsystem, Modes.ALIGN_HIGH, Modes.OFF),
        parallel(
            new FollowTrajectoryCommand(threeBallOnsidePickup, true, drivebaseSubsystem),
            new ForceIntakeModeCommand(intakeSubsystem, Modes.INTAKE),
            armAngleCommand.apply(Arm.Setpoints.INTAKE_POSITION)),
        new SequentialCommandGroup(
            armAngleCommand.apply(Arm.Setpoints.OUTTAKE_HIGH_POSITION), new WaitCommand(.2)),
        new FollowTrajectoryCommand(threeBallOnsideShoot, drivebaseSubsystem),
        new SetIntakeModeCommand(intakeSubsystem, Modes.ALIGN_HIGH, Modes.OFF));
  }
}
