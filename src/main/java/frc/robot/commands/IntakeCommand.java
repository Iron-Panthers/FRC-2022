// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * A command which handles the intake lifecycle. Should be equal to the inlined code below.
 *
 * <pre>
 * Command myIntakeCommand =
 *     new StartEndCommand(
 *         () -> intakeSubsystem.setMode(IntakeSubsystem.Modes.YOUR_MODE_HERE),
 *         intakeSubsystem::nextMode,
 *         intakeSubsystem);
 * </pre>
 */
public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem.Modes currentMode;
  private final IntakeSubsystem intakeSubsystem;

  /** A responsible intake command that handles cleanup using the nextState provider */
  public IntakeCommand(IntakeSubsystem.Modes currentMode, IntakeSubsystem intakeSubsystem) {
    this.currentMode = currentMode;
    this.intakeSubsystem = intakeSubsystem;
    addRequirements(intakeSubsystem);
  }

  @Override
  public void initialize() {
    // Write the desired mode at present
    intakeSubsystem.setMode(currentMode);
  }

  @Override
  public void end(boolean interrupted) {
    // Do hand-off to next mode in progression
    intakeSubsystem.nextMode();
  }
}
