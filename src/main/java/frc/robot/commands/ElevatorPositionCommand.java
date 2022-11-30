// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.fasterxml.jackson.core.filter.FilteringParserDelegate;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorPositionCommand extends CommandBase {
  private final ElevatorSubsystem elevatorSubsystem;
  private final Double targetHeight;
  private double startTime = 0;

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
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevatorSubsystem.setNeutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Timer.getFPGATimestamp() < startTime + 1) return false;
    return elevatorSubsystem.atTarget();
  }
}

// 12.75 full motor rotations = 1.5pi inches of height

// 12.75:1 gear ratio
// Big gear is 2.6 inches
// 1.5 sprocket diameter
