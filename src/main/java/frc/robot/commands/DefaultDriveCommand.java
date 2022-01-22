// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivebaseSubsystem;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
  private final DrivebaseSubsystem drivebaseSubsystem;

  private final DoubleSupplier translationXSupplier;
  private final DoubleSupplier translationYSupplier;
  private final DoubleSupplier rotationSupplier;

  /** Creates a new DefaultDriveCommand. */
  public DefaultDriveCommand(
      DrivebaseSubsystem drivebaseSubsystem,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {

    this.drivebaseSubsystem = drivebaseSubsystem;
    this.translationXSupplier = translationXSupplier;
    this.translationYSupplier = translationYSupplier;
    this.rotationSupplier = rotationSupplier;

    addRequirements(drivebaseSubsystem);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: scaled radial deadzones rather than hard cutoff
    double x = translationXSupplier.getAsDouble();
    if (Math.abs(x) < 0.2)
      x = 0;
    double y = translationYSupplier.getAsDouble();
    if (Math.abs(y) < 0.2)
      y = 0;
    double rot = rotationSupplier.getAsDouble();
    if (Math.abs(rot) < 0.2)
      rot = 0;

    if (Math.abs(x) < 0.2 && Math.abs(y) < 0.2 && Math.abs(rot) < 0.2) {
      drivebaseSubsystem.setNeutral();
      return;
    }

    // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented
    // movement
    drivebaseSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            translationXSupplier.getAsDouble(),
            translationYSupplier.getAsDouble(),
            rotationSupplier.getAsDouble(),
            drivebaseSubsystem.getGyroscopeRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivebaseSubsystem.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
