// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import java.util.function.DoubleSupplier;

public class PreciseArmCommand extends CommandBase {
  private ArmSubsystem armSubsystem;

  private double target;

  private DoubleSupplier stick;

  /** Creates a new PreciseArmCommand. */
  public PreciseArmCommand(ArmSubsystem armSubsystem, DoubleSupplier stick) {
    this.armSubsystem = armSubsystem;
    this.stick = stick;
    addRequirements(armSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = armSubsystem.getAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target += +(.15 * stick.getAsDouble());
    armSubsystem.setAngle(target);
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
