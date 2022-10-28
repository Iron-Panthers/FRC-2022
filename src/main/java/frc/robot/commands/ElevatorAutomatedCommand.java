// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

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
  public ElevatorAutomatedCommand(ElevatorSubsystem subsystem) {
    this.elevatorSubsystem = subsystem;
    armSubsystem = new ArmSubsystem(subsystem);
    addRequirements(elevatorSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.

    addCommands(
        new SequentialCommandGroup(
            // mid rung
            new ElevatorPositionCommand(subsystem, Elevator.MIN_HEIGHT),
            new WaitCommand(3),
            // high rung
            new ElevatorPositionCommand(subsystem, Elevator.MAX_HEIGHT),
            new WaitCommand(1),
            new ElevatorPositionCommand(subsystem, Elevator.MIN_HEIGHT),
            parallel(
                new WaitCommand(3),
                new InstantCommand(
                    () -> armSubsystem.setAngle(Arm.Setpoints.CLIMB_POSITION), armSubsystem)),
            // traversal rung
            new ElevatorPositionCommand(subsystem, Elevator.MAX_HEIGHT),
            new WaitCommand(3),
            parallel(
                deadline(
                    new WaitCommand(5),
                    new ElevatorPositionCommand(subsystem, Elevator.MIN_HEIGHT)))));
  }
}
