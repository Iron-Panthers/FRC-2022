package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetIntakeModeCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.Modes;

public class ShootNoTaxi extends SequentialCommandGroup {
  public ShootNoTaxi(IntakeSubsystem intakeSubsystem) {

    addCommands(new SetIntakeModeCommand(intakeSubsystem, Modes.CENTER_NORMALIZE_HIGH, Modes.OFF));
  }
}
