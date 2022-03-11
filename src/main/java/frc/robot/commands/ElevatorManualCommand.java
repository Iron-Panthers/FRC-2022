// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorManualCommand extends CommandBase {
  private ElevatorSubsystem elevatorSubsystem;
  private final Double rate;
  /** Creates a new ElevatorCommand. */
  public ElevatorManualCommand(ElevatorSubsystem subsystem, Double rate) {
    this.rate = rate;
    this.elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("rate", rate);
    elevatorSubsystem.setTargetHeight(elevatorSubsystem.getTargetHeight() + rate);
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
