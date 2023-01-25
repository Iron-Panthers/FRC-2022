package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.SetIntakeModeCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;

public class OffsideTaxiShot extends SequentialCommandGroup {
  public OffsideTaxiShot(
      // for some reason we don't use this value lol
      // changing that would require testing, boo
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      SwerveDriveKinematics kinematics,
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem) {

    PathPlannerTrajectory taxiShotPath =
        PathPlanner.loadPath(
            "taxiOffsideShotPath",
            maxAccelerationMetersPerSecondSq,
            maxAccelerationMetersPerSecondSq);

    // DoubleFunction<InstantCommand> armAngleCommand =
    //     angle -> new InstantCommand(() -> armSubsystem.setAngle(angle), armSubsystem);

    addCommands(
        new SetIntakeModeCommand(intakeSubsystem, Modes.CENTER_NORMALIZE_HIGH, Modes.OFF),
        new FollowTrajectoryCommand(taxiShotPath, true, drivebaseSubsystem));
  }
}
