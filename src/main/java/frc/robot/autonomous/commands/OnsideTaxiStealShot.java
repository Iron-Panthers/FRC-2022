package frc.robot.autonomous.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.Arm;
import frc.robot.commands.FollowTrajectoryCommand;
import frc.robot.commands.ForceIntakeModeCommand;
import frc.robot.commands.SetIntakeModeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;
import java.util.function.DoubleFunction;

public class OnsideTaxiStealShot extends SequentialCommandGroup {
  public OnsideTaxiStealShot(
      double maxVelocityMetersPerSecond,
      double maxAccelerationMetersPerSecondSq,
      ArmSubsystem armSubsystem,
      DrivebaseSubsystem drivebaseSubsystem,
      IntakeSubsystem intakeSubsystem) {

    PathPlannerTrajectory taxiStealPath =
        PathPlanner.loadPath(
            "taxiOnsideStealShotPath",
            maxVelocityMetersPerSecond,
            maxAccelerationMetersPerSecondSq);

    DoubleFunction<InstantCommand> armAngleCommand =
        angle -> new InstantCommand(() -> armSubsystem.setAngle(angle), armSubsystem);

    addCommands(
        // take shot
        new SetIntakeModeCommand(intakeSubsystem, Modes.CENTER_NORMALIZE_HIGH, Modes.OFF),
        deadline(
            // drive the path while putting the arm down
            parallel(
                new FollowTrajectoryCommand(taxiStealPath, true, drivebaseSubsystem),
                armAngleCommand.apply(Arm.Setpoints.INTAKE_POSITION)),
            // run the intake until the path ends
            new ForceIntakeModeCommand(intakeSubsystem, Modes.INTAKE)));
  }
}
