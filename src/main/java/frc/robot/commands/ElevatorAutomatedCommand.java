// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Function;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.Arm;
import frc.robot.Constants.Elevator;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorAutomatedCommand extends SequentialCommandGroup {
  private final ElevatorSubsystem elevatorSubsystem;
  private final ArmSubsystem armSubsystem;

  /** Creates a new ElevatorCommand. */
  public ElevatorAutomatedCommand(ElevatorSubsystem elevatorSubsystem, ArmSubsystem armSubsystem) {
    this.elevatorSubsystem = elevatorSubsystem;
    this.armSubsystem = armSubsystem;
    addRequirements(elevatorSubsystem, armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
      new SequentialCommandGroup(
        new InstantCommand(
          () -> armSubsystem.setAngle(Arm.Setpoints.CLIMB_POSITION),
          armSubsystem
        ),
        // hook on to mid rung
        new ElevatorPositionCommand(elevatorSubsystem, Elevator.MIN_HEIGHT),
        new ElevatorPositionCommand(elevatorSubsystem, Elevator.HOOK_ENGAGED_HEIGHT),
        new WaitCommand(1),
        // extend to high rung
        new ElevatorPositionCommand(elevatorSubsystem, Elevator.SEQUENCE_EXTEND_HEIGHT),
        // hook on to high rung
        new ElevatorPositionCommand(elevatorSubsystem, Elevator.MIN_HEIGHT),
        new ElevatorPositionCommand(elevatorSubsystem, Elevator.HOOK_ENGAGED_HEIGHT),
        new WaitCommand(1),
        // extend to traversal rung
        new ElevatorPositionCommand(elevatorSubsystem, Elevator.SEQUENCE_EXTEND_HEIGHT),
        new WaitCommand(0.5),
        // hook on to traversal rung
        new ElevatorPositionCommand(elevatorSubsystem, Elevator.MIN_HEIGHT)));
  }
}
