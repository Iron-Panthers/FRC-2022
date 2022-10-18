// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.commands.ElevatorPositionCommand;
import frc.robot.Constants.Elevator;

public class ElevatorAutomatedCommand extends SequentialCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;

  /** Creates a new ElevatorCommand. */
  public ElevatorAutomatedCommand(ElevatorSubsystem subsystem, double targetHeight) {
    this.elevatorSubsystem = subsystem;
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      new SequentialCommandGroup(
        // mid rung
        new ElevatorPositionCommand(subsystem, Elevator.MAX_HEIGHT);
        new WaitCommand(0.2);
        new ElevatorPositionCommand(subsystem, Elevator.MIN_HEIGHT);
        new WaitCommand(0.7);
        // high rung
        new ElevatorPositionCommand(subsystem, Elevator.MAX_HEIGHT);
        new WaitCommand(0.2);
        new ElevatorPositionCommand(subsystem, Elevator.MIN_HEIGHT);
        new WaitCommand(0.7);
        // traversal rung
        new ElevatorPositionCommand(subsystem, Elevator.MAX_HEIGHT);
        new WaitCommand(0.2);
        new ElevatorPositionCommand(subsystem, Elevator.MIN_HEIGHT);
        new WaitCommand(0.7);
      )
    );
  }
}