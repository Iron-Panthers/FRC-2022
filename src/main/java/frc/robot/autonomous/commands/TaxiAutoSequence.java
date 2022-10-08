package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class TaxiAutoSequence extends SequentialCommandGroup {
  public TaxiAutoSequence(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      SwerveDriveKinematics kinematics,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem) {

    // TrajectoryConfig trajectoryConfig =
    //     new TrajectoryConfig(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq)
    //         .setKinematics(kinematics);

    PathPlannerTrajectory path =
        PathPlanner.loadPath(
            "taxi", maxAccelerationMetersPerSecondSq, maxAccelerationMetersPerSecondSq);

    addCommands(new FollowTrajectoryCommand(path, true, drivebaseSubsystem));
  }
}