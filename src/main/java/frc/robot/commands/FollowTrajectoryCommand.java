// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;

public class FollowTrajectoryCommand extends CommandBase {
  private final DrivebaseSubsystem drivebaseSubsystem;
  private final Trajectory trajectory;

  /** Creates a new FollowTrajectoryCommand. */
  public FollowTrajectoryCommand(Trajectory trajectory, DrivebaseSubsystem drivebaseSubsystem) {
    this.trajectory = trajectory;
    this.drivebaseSubsystem = drivebaseSubsystem;
    addRequirements(drivebaseSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivebaseSubsystem.getFollower().follow(trajectory);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.getFollower().cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return drivebaseSubsystem.getFollower().getCurrentTrajectory().isEmpty();
  }
}
