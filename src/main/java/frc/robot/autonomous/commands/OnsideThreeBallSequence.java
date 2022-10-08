package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.SetIntakeModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;

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

    addCommands(
        new SetIntakeModeCommand(intakeSubsystem, Modes.ALIGN_HIGH, Modes.OFF),
        new FollowTrajectoryCommand(threeBallOnsidePickup, true, drivebaseSubsystem));
  }
}
