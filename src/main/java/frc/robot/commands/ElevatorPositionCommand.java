// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final Double targetHeight;

  /** Creates a new ElevatorCommand. */
  public ElevatorPositionCommand(ElevatorSubsystem subsystem, Double targetHeight) {
    this.elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
    this.targetHeight = targetHeight;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevatorSubsystem.setTargetHeight(targetHeight);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get current height
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// 12.75 full motor rotations = 1.5pi inches of height

// 12.75:1 gear ratio
// talonfx to the entire thing
// Big gear is 2.6 inches
// 1.5 sprocket diameter
